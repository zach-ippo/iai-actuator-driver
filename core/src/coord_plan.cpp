// Coordinated motion: axis ranges, line timing, the planner (feasibility before any frame, the largest feasible speed
// by bisection, merging of too-short steps, chord-error stepping), plans and results (coord.hpp). Port of the Python
// edition's coord.py (AxisRange, LineTiming, MotionPlan, plan, _plan_sync, _plan_stream, _merge_steps, _Checks,
// path_deviation, _merge, _spacing, CoordinatedResult). Pure: no I/O. UNVERIFIED on hardware in this edition.
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "iai_actuator_driver/coord.hpp"
#include "util.hpp"

namespace iai_actuator_driver {
namespace coord {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

std::string g6(double v) { return util::format("%g", v); }
std::string f(double v, int decimals) { return util::fixed(v, decimals); }

double round_to(double v, int decimals) {
    const double scale = std::pow(10.0, decimals);
    return std::round(v * scale) / scale;
}

Json rounded(const std::optional<double>& v, int decimals) {
    return v ? Json(round_to(*v, decimals)) : Json();
}

double number(double value, const std::string& what) {
    if (!std::isfinite(value)) throw std::invalid_argument(what + " must be a finite number, got " + g6(value));
    return value;
}

double positive(double value, const std::string& what) {
    number(value, what);
    if (value <= 0) throw std::invalid_argument(what + " must be > 0, got " + g6(value));
    return value;
}

double floor2(double v) { return std::floor(v * 100 + 1e-9) / 100; }
double ceil2(double v) { return std::ceil(v * 100 - 1e-9) / 100; }
double round2(double v) { return std::round(v * 100) / 100; }

std::int64_t counts_of(double v) { return reg::to_counts(v); }

double dist(const std::vector<double>& a, const std::vector<double>& b) {
    double sum = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) sum += (b[i] - a[i]) * (b[i] - a[i]);
    return std::sqrt(sum);
}

struct Profile {
    double peak = 0.0;
    double t_ramp = 0.0;
    double s_ramp = 0.0;
    double total = 0.0;
};

/// (peak speed, ramp time, ramp length, total time) of a trapezoid from rest to rest over `length`.
Profile profile(double length, double speed, double accel) {
    if (length <= 0) return {};
    const double ramp = speed * speed / (2 * accel);
    if (2 * ramp >= length) {
        const double peak = std::sqrt(accel * length);
        return {peak, peak / accel, length / 2, 2 * peak / accel};
    }
    return {speed, speed / accel, ramp, 2 * speed / accel + (length - 2 * ramp) / speed};
}

/// When the trapezoid over `length` passes `s`.
double time_at(double s, double length, double speed, double accel) {
    const Profile p = profile(length, speed, accel);
    if (s <= 0) return 0.0;
    if (s >= length) return p.total;
    if (s < p.s_ramp) return std::sqrt(2 * s / accel);
    if (s <= length - p.s_ramp) return p.t_ramp + (s - p.s_ramp) / p.peak;
    return p.total - std::sqrt(2 * (length - s) / accel);
}

/// Collects the refusals of one plan and, per refusal, a first estimate of the largest speed that would pass.
struct Checks {
    explicit Checks(double speed_) : speed(speed_) {}
    double speed;
    std::vector<std::string> problems;
    std::vector<double> feasible;
    bool no_speed_helps = false;

    void add(const std::string& text, std::optional<double> feasible_speed = std::nullopt) {
        problems.push_back(text);
        if (!feasible_speed)
            no_speed_helps = true;
        else
            feasible.push_back(std::max(0.0, *feasible_speed));
    }
    /// The linear estimate of the largest feasible speed (exact for sync plans, a first guess for stream ones).
    std::optional<double> estimate() const {
        if (no_speed_helps || feasible.empty()) return std::nullopt;
        return floor2(*std::min_element(feasible.begin(), feasible.end()));
    }
    InfeasiblePathError error(std::optional<double> best) const {
        const std::string tail = best ? "the largest feasible speed is about " + g6(*best) + " (asked " + g6(speed) + ")"
                                      : std::string("no speed makes this path feasible");
        return InfeasiblePathError(
            "coordinated move refused, nothing sent: " + util::join(problems, "; ") + "; " + tail, problems, best);
    }
};

/// A plan's checks failed (internal: plan() turns it into an InfeasiblePathError).
struct Infeasible {
    Checks checks;
};

void raise_if_any(const Checks& checks) {
    if (!checks.problems.empty()) throw Infeasible{checks};
}

/// Every commanded waypoint (points[1:]; points[0] is where the axes already are, never sent) inside each axis's range.
void check_positions(Checks& checks, const std::vector<std::string>& axes,
                     const std::vector<std::vector<double>>& points, const std::map<std::string, AxisRange>& ranges) {
    for (std::size_t i = 0; i < axes.size(); ++i) {
        const auto it = ranges.find(axes[i]);
        if (it == ranges.end() || !it->second.position || points.size() < 2) continue;
        const double lo = it->second.position->min;
        const double hi = it->second.position->max;
        double mn = kInf, mx = -kInf;
        for (std::size_t k = 1; k < points.size(); ++k) {
            mn = std::min(mn, points[k][i]);
            mx = std::max(mx, points[k][i]);
        }
        std::optional<double> worst;
        if (mn < lo - 1e-9)
            worst = mn;
        else if (mx > hi + 1e-9)
            worst = mx;
        if (!worst) continue;
        std::size_t k = 1;
        while (k < points.size() && points[k][i] != *worst) ++k;
        checks.add("axis " + axes[i] + ": waypoint " + std::to_string(k) + " at " + f(*worst, 3) +
                   " is outside its range " + g6(lo) + ".." + g6(hi) +
                   " (the soft limits minus their margin, within a configured stroke)");
    }
}

/// Per-axis speed (refusal), accel (refusal for sync; for stream the path accel is checked instead) and the documented
/// minimum speed (one warning per axis).
std::vector<std::string> check_axis_rates(Checks& checks, const std::vector<std::string>& axes,
                                          const std::vector<std::vector<double>>& speeds,
                                          const std::vector<std::vector<double>>& accels,
                                          const std::map<std::string, AxisRange>& ranges, double accel_g,
                                          bool frame_accel) {
    std::vector<std::string> warnings;
    for (std::size_t i = 0; i < axes.size(); ++i) {
        const auto it = ranges.find(axes[i]);
        const AxisRange rng = it == ranges.end() ? AxisRange() : it->second;
        std::vector<double> moving;
        for (const auto& s : speeds)
            if (s[i] > 0) moving.push_back(s[i]);
        if (moving.empty()) continue;
        const double top = *std::max_element(moving.begin(), moving.end());
        if (rng.max_speed && top > *rng.max_speed + 1e-9)
            checks.add("axis " + axes[i] + ": " + g6(top) + " " + rng.units + "/s above its maximum " +
                           g6(*rng.max_speed),
                       checks.speed * *rng.max_speed / top);
        if (rng.max_accel_g) {
            const double cap = *rng.max_accel_g;
            if (frame_accel) {
                if (accel_g > cap + 1e-9)
                    checks.add("axis " + axes[i] + ": path acceleration " + g6(accel_g) + " G above its maximum " +
                               g6(cap) + " G (lower accel_g)");
            } else {
                double top_a = 0.0;
                for (const auto& a : accels) top_a = std::max(top_a, a[i]);
                if (top_a > cap + 1e-9)
                    checks.add("axis " + axes[i] + ": acceleration " + g6(top_a) + " G above its maximum " + g6(cap) +
                               " G (lower accel_g)");
            }
        }
        const double low = *std::min_element(moving.begin(), moving.end());
        if (rng.min_speed && low < *rng.min_speed - 1e-9)
            warnings.push_back("axis " + axes[i] + ": speeds down to " + g6(low) + " " + rng.units +
                               "/s, below its documented minimum " + g6(*rng.min_speed) + " " + rng.units +
                               "/s (allowed; it may run unsteadily there)");
    }
    return warnings;
}

/// The largest speed (0.01 steps) at which `attempt` passes every check, by bisection over the planner itself:
/// per-axis speeds and v^2 / r rise with the speed and step times fall, so feasibility is monotone. nullopt when not
/// even 0.01 passes.
std::optional<double> largest_speed(const std::function<MotionPlan(double)>& attempt, double speed,
                                    std::optional<double> estimate) {
    auto ok = [&](double v) {
        try {
            attempt(v);
        } catch (const Infeasible&) {
            return false;
        }
        return true;
    };
    std::optional<double> start;
    if (estimate && 0.01 <= *estimate && *estimate < speed && ok(*estimate))
        start = estimate;
    else if (ok(0.01))
        start = 0.01;
    if (!start) return std::nullopt;
    double lo = *start;
    double hi = speed;
    while (hi - lo > 0.004) {
        const double mid = (lo + hi) / 2;
        if (ok(mid))
            lo = mid;
        else
            hi = mid;
    }
    double best = floor2(lo);
    for (int i = 0; i < 5; ++i) {  // rounding: step down until it passes
        if (best < 0.01) return std::nullopt;
        if (ok(best)) return best;
        best = round2(best - 0.01);
    }
    return std::nullopt;
}

struct Common {
    double accel_g = 0.1;
    double chord_error = 0.05;
    double band = 0.1;
    std::map<std::string, AxisRange> ranges;
    LineTiming timing;
    double min_step = 0.0;
};

MotionPlan make_plan(Mode mode, const std::vector<std::string>& axes, std::vector<std::vector<double>> points,
                     std::vector<double> times, std::vector<std::vector<double>> speeds,
                     std::vector<std::vector<double>> accels, double speed, const Common& c, double length,
                     double est_chord, double skew, double est_dev, std::optional<double> blend,
                     std::vector<std::string> warnings) {
    MotionPlan p;
    p.mode = mode;
    p.axes = axes;
    p.points = std::move(points);
    p.times = std::move(times);
    p.speeds = std::move(speeds);
    p.accels = std::move(accels);
    p.path_speed = speed;
    p.accel_g = c.accel_g;
    p.chord_error = c.chord_error;
    p.length = length;
    p.band = c.band;
    p.min_step_s = c.min_step;
    p.est_chord_error = est_chord;
    p.est_timing_error_s = skew;
    p.est_deviation = est_dev;
    p.blend = blend;
    p.warnings = std::move(warnings);
    p.timing = c.timing;
    p.ranges = c.ranges;
    return p;
}

MotionPlan plan_sync(const std::vector<Piece>& pieces, const std::vector<std::string>& axes, double speed,
                     const Common& c, std::optional<double> blend) {
    const double accel = c.accel_g * kG;
    std::vector<std::vector<double>> points;
    {
        std::vector<double> first;
        for (const auto& a : axes) first.push_back(pieces[0].start.at(a));
        points.push_back(first);
    }
    for (const auto& p : pieces) {
        std::vector<double> row;
        for (const auto& a : axes) row.push_back(p.end.at(a));
        points.push_back(row);
    }
    std::vector<double> times{0.0};
    std::vector<std::vector<double>> speeds, accels;
    std::vector<std::string> warnings;
    std::set<std::string> rounded_axes;
    double total_length = 0.0;
    for (const auto& p : pieces) {
        std::vector<double> d;
        double sum = 0.0;
        for (const auto& a : axes) {
            d.push_back(p.end.at(a) - p.start.at(a));
            sum += d.back() * d.back();
        }
        const double length = std::sqrt(sum);
        total_length += p.length();
        times.push_back(times.back() + profile(length, speed, accel).total);
        std::vector<double> sp, ac;
        for (std::size_t i = 0; i < axes.size(); ++i) {
            const std::string& axis = axes[i];
            if (counts_of(p.end.at(axis)) == counts_of(p.start.at(axis))) {
                sp.push_back(0.0);
                ac.push_back(0.0);
                continue;
            }
            const double share = std::fabs(d[i]) / length;
            sp.push_back(std::max(0.01, round2(speed * share)));
            const double a_i = std::max(0.01, round2(c.accel_g * share));
            if (std::fabs(a_i - c.accel_g * share) > 0.1 * c.accel_g * share && !rounded_axes.count(axis)) {
                rounded_axes.insert(axis);
                warnings.push_back("axis " + axis + ": acceleration " + f(c.accel_g * share, 4) + " G rounds to " +
                                   g6(a_i) + " G (0.01 G steps): its ramps differ a little from the others, the path "
                                   "bends slightly there");
            }
            ac.push_back(a_i);
        }
        speeds.push_back(sp);
        accels.push_back(ac);
    }
    Checks checks(speed);
    check_positions(checks, axes, points, c.ranges);
    const auto more = check_axis_rates(checks, axes, speeds, accels, c.ranges, c.accel_g, false);
    warnings.insert(warnings.end(), more.begin(), more.end());
    raise_if_any(checks);
    const double skew = kJitterS + std::max(0, c.timing.axes_on_port - 1) * c.timing.write_s();
    return make_plan(Mode::Sync, axes, std::move(points), std::move(times), std::move(speeds), std::move(accels), speed,
                     c, total_length, 0.0, skew, speed * skew, blend, std::move(warnings));
}

/// The waypoints a streamed plan keeps (indices) and the largest corner cut. A waypoint whose counts equal the last
/// kept one's on every axis sends no frame: it is dropped and its step merges into the next (the last waypoint too:
/// the end then differs from the path's by less than one count). A step shorter than min_step merges with its
/// neighbour across a waypoint inside a straight piece (`inner`: collinear) or, when the step is no longer than
/// max_cut (the chord error), across any waypoint: the path then cuts that corner by less than the step's length.
std::pair<std::vector<std::size_t>, double> merge_steps(const std::vector<double>& times,
                                                        const std::vector<double>& dists,
                                                        const std::vector<std::vector<std::int64_t>>& counts,
                                                        std::vector<bool> inner, double min_step, double max_cut) {
    std::vector<std::size_t> keep{0};
    for (std::size_t k = 1; k < times.size(); ++k)
        if (counts[k] != counts[keep.back()]) keep.push_back(k);
    double cut = 0.0;
    if (min_step > 0) {
        std::size_t i = 1;
        while (i < keep.size()) {
            const std::size_t a = keep[i - 1];
            const std::size_t b = keep[i];
            const double length = dists[b] - dists[a];
            if (times[b] - times[a] < min_step - 1e-9) {
                bool merged = false;
                const std::pair<std::size_t, std::size_t> candidates[2] = {{i, b}, {i - 1, a}};
                for (const auto& cand : candidates) {
                    const std::size_t j = cand.first;
                    const std::size_t drop = cand.second;
                    if (!(0 < j && j < keep.size() - 1) || !(inner[drop] || length <= max_cut)) continue;
                    if (!inner[drop]) {  // a corner cut: its neighbours are no longer collinear with theirs
                        cut = std::max(cut, length);
                        inner[keep[j - 1]] = false;
                        inner[keep[j + 1]] = false;
                    }
                    keep.erase(keep.begin() + static_cast<std::ptrdiff_t>(j));
                    i = std::max<std::size_t>(1, j);
                    merged = true;
                    break;
                }
                if (!merged) ++i;
                continue;
            }
            ++i;
        }
    }
    return {keep, cut};
}

MotionPlan plan_stream(const std::vector<Piece>& pieces, const std::vector<std::string>& axes, double speed,
                       const Common& c, std::optional<double> stream_accel_g) {
    const double accel = c.accel_g * kG;
    const double max_piece = speed * std::max(kLineStepS, 2 * c.min_step);
    std::vector<std::vector<double>> points;
    {
        std::vector<double> first;
        for (const auto& a : axes) first.push_back(pieces[0].start.at(a));
        points.push_back(first);
    }
    std::vector<double> radii;      // the radius each step runs on (inf: straight)
    std::vector<bool> inner{false};  // per waypoint: inside a straight piece (collinear with both neighbours)
    double est_chord = 0.0;
    for (const auto& p : pieces) {
        int n = 1;
        double radius = kInf;
        if (p.straight()) {
            n = std::max(1, static_cast<int>(std::ceil(p.length() / max_piece - 1e-9)));
        } else {
            radius = std::min(p.r0, p.r1);
            if (!(radius >= kMinRadius))  // Path::pieces refuses these already
                throw std::invalid_argument("an arc radius " + g6(radius) + " below one count (" + g6(kMinRadius) +
                                            ") cannot be streamed");
            const double e = std::min(c.chord_error, radius);
            const double step = 2 * std::acos(1 - e / radius);
            n = std::max({1, static_cast<int>(std::ceil(std::fabs(p.sweep) / step - 1e-9)),
                          static_cast<int>(std::ceil(std::fabs(p.sweep) / kMaxChordAngle - 1e-9))});
            est_chord = std::max(est_chord, std::max(p.r0, p.r1) * (1 - std::cos(std::fabs(p.sweep) / n / 2)));
        }
        for (int j = 1; j <= n; ++j) {
            const Point q = p.at(static_cast<double>(j) / n);
            std::vector<double> row;
            for (const auto& a : axes) row.push_back(q.at(a));
            points.push_back(row);
            radii.push_back(radius);
            inner.push_back(p.straight() && j < n);
        }
    }
    std::vector<double> s{0.0};
    for (std::size_t k = 1; k < points.size(); ++k) s.push_back(s.back() + dist(points[k - 1], points[k]));
    const double total = s.back();
    std::vector<double> times;
    for (double x : s) times.push_back(time_at(x, total, speed, accel));
    times.back() = profile(total, speed, accel).total;
    std::vector<std::vector<std::int64_t>> counts;
    for (const auto& p : points) {
        std::vector<std::int64_t> row;
        for (double v : p) row.push_back(counts_of(v));
        counts.push_back(row);
    }
    const auto merged = merge_steps(times, s, counts, inner, c.min_step, c.chord_error);
    const std::vector<std::size_t>& keep = merged.first;
    est_chord = std::max(est_chord, merged.second);
    if (keep.size() < 2) throw NoMotionError("the path moves no axis by a whole position count (0.01)");
    {
        std::vector<double> kept_radii;
        for (std::size_t m = 0; m + 1 < keep.size(); ++m)
            kept_radii.push_back(*std::min_element(radii.begin() + static_cast<std::ptrdiff_t>(keep[m]),
                                                   radii.begin() + static_cast<std::ptrdiff_t>(keep[m + 1])));
        radii = kept_radii;
    }
    {
        std::vector<std::vector<double>> kp;
        std::vector<double> kt;
        std::vector<std::vector<std::int64_t>> kc;
        for (std::size_t k : keep) {
            kp.push_back(points[k]);
            kt.push_back(times[k]);
            kc.push_back(counts[k]);
        }
        points = kp;
        times = kt;
        counts = kc;
    }
    std::vector<std::vector<double>> speeds, accels;
    std::vector<std::pair<std::string, double>> capped;  // in order of appearance
    std::vector<double> previous(axes.size(), 0.0);
    for (std::size_t k = 0; k + 1 < points.size(); ++k) {
        const double dt = times[k + 1] - times[k];
        std::vector<double> sp, ac;
        for (std::size_t i = 0; i < axes.size(); ++i) {
            const std::string& axis = axes[i];
            const double delta = static_cast<double>(std::llabs(counts[k + 1][i] - counts[k][i])) * reg::kPositionScale;
            if (delta == 0) {
                sp.push_back(0.0);
                ac.push_back(0.0);
                previous[i] = 0.0;
                continue;
            }
            const double v = std::max(0.01, round2(delta / dt));
            const double need =
                stream_accel_g ? *stream_accel_g : std::max(4 * std::fabs(v - previous[i]), 8 * v) / dt / kG;
            double a_i = std::min(reg::kAcmdMaxRaw * reg::kAccelScale, std::max(0.01, ceil2(need)));  // ACMD: 3.00 G
            const auto rit = c.ranges.find(axis);
            const std::optional<double> cap = rit == c.ranges.end() ? std::nullopt : rit->second.max_accel_g;
            if (cap && a_i > *cap) {
                auto it = std::find_if(capped.begin(), capped.end(), [&](const auto& kv) { return kv.first == axis; });
                if (it == capped.end())
                    capped.emplace_back(axis, need);
                else
                    it->second = std::max(it->second, need);
                a_i = std::max(0.01, floor2(*cap));
            }
            sp.push_back(v);
            ac.push_back(a_i);
            previous[i] = v;
        }
        speeds.push_back(sp);
        accels.push_back(ac);
    }
    Checks checks(speed);
    check_positions(checks, axes, points, c.ranges);
    std::vector<std::string> warnings = check_axis_rates(checks, axes, speeds, accels, c.ranges, c.accel_g, true);
    std::vector<double> caps;
    for (std::size_t i = 0; i < axes.size(); ++i) {
        bool moving = false;
        for (const auto& sp : speeds)
            if (sp[i] > 0) moving = true;
        const auto rit = c.ranges.find(axes[i]);
        if (moving && rit != c.ranges.end() && rit->second.max_accel_g) caps.push_back(*rit->second.max_accel_g);
    }
    if (!caps.empty()) {
        const double cap_g = *std::min_element(caps.begin(), caps.end());
        std::optional<std::pair<double, double>> worst;  // (v, r)
        for (std::size_t k = 0; k < radii.size(); ++k) {
            const double r = radii[k];
            if (std::isinf(r)) continue;
            const double v = dist(points[k], points[k + 1]) / (times[k + 1] - times[k]);
            if (v * v / r > cap_g * kG + 1e-9 && (!worst || r < worst->second)) worst = std::make_pair(v, r);
        }
        if (worst) {
            const double v = worst->first;
            const double r = worst->second;
            checks.add("centripetal acceleration " + f(v * v / r / kG, 3) + " G on radius " + g6(r) + " at " + f(v, 2) +
                           "/s is above the smallest axis acceleration " + g6(cap_g) + " G",
                       std::sqrt(cap_g * kG * r));
        }
    }
    double shortest = kInf;
    for (std::size_t k = 0; k + 1 < times.size(); ++k) shortest = std::min(shortest, times[k + 1] - times[k]);
    if (c.min_step > 0 && shortest < c.min_step - 1e-9)
        checks.add("steps down to " + f(shortest * 1000, 1) + " ms, shorter than the " + f(c.min_step * 1000, 1) +
                       " ms the line can carry (lower the speed, raise chord_error, or a faster baud rate)",
                   speed * shortest / c.min_step);
    raise_if_any(checks);
    for (const auto& kv : capped) {
        const double cap = *c.ranges.at(kv.first).max_accel_g;
        warnings.push_back("axis " + kv.first + ": re-targeting wants up to " + f(kv.second, 2) +
                           " G, capped at its maximum " + g6(cap) + " G: it lags behind the schedule there");
    }
    const double skew = kJitterS + std::max(0, c.timing.axes_on_port - 1) * c.timing.write_s();
    return make_plan(Mode::Stream, axes, std::move(points), std::move(times), std::move(speeds), std::move(accels),
                     speed, c, total, est_chord, skew, est_chord + speed * skew, std::nullopt, std::move(warnings));
}

double segment_distance(const std::vector<double>& p, const std::vector<double>& a, const std::vector<double>& b) {
    std::vector<double> ab;
    double l2 = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        ab.push_back(b[i] - a[i]);
        l2 += ab.back() * ab.back();
    }
    if (l2 == 0) return dist(p, a);
    double dot = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) dot += (p[i] - a[i]) * ab[i];
    const double t = std::max(0.0, std::min(1.0, dot / l2));
    std::vector<double> q;
    for (std::size_t i = 0; i < a.size(); ++i) q.push_back(a[i] + t * ab[i]);
    return dist(p, q);
}

double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    const std::size_t n = values.size();
    if (n == 0) return 0.0;
    return n % 2 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2;
}

}  // namespace

// ============================================================================================ AxisRange

AxisRange AxisRange::of(const AxisRanges& limits) {
    AxisRange r;
    if (limits.position.min && limits.position.max) r.position = Span{*limits.position.min, *limits.position.max};
    r.max_speed = limits.speed.max;
    r.max_accel_g = limits.accel_decel.max ? limits.accel_decel.max : limits.accel.max;
    r.min_speed = limits.speed.recommended_min;
    r.units = limits.units.empty() ? "mm" : limits.units;
    return r;
}

AxisRange AxisRange::make(double min, double max, std::optional<double> max_speed, std::optional<double> max_accel_g,
                          std::optional<double> min_speed) {
    AxisRange r;
    r.position = Span{min, max};
    r.max_speed = max_speed;
    r.max_accel_g = max_accel_g;
    r.min_speed = min_speed;
    return r;
}

Json AxisRange::to_json() const {
    Json j = Json::object();
    if (position) {
        Json pos = Json::array();
        pos.push_back(position->min);
        pos.push_back(position->max);
        j.set("position", pos);
    } else {
        j.set("position", Json());
    }
    j.set("max_speed", max_speed);
    j.set("max_accel_g", max_accel_g);
    j.set("min_speed", min_speed);
    j.set("units", units);
    return j;
}

bool AxisRange::operator==(const AxisRange& o) const {
    return position == o.position && max_speed == o.max_speed && max_accel_g == o.max_accel_g &&
           min_speed == o.min_speed && units == o.units;
}

// ============================================================================================ LineTiming

double LineTiming::frame_s(int request_bytes, int response_bytes) const {
    const double wire = 10.0 * (request_bytes + response_bytes) / std::max(1, baud_rate);
    double reply = response_ms / 1000.0;
    if (baud_rate < kSlowBaud) reply = std::max(reply, kSlowReplyS);
    return wire + reply + gap_s + turnaround_s;
}

double LineTiming::write_s() const {
    return std::max(frame_s(kWriteRequestBytes, kWriteResponseBytes), measured_write_s.value_or(0.0));
}

double LineTiming::read_s() const {
    return std::max(frame_s(kReadRequestBytes, kReadResponseBytes), measured_read_s.value_or(0.0));
}

double LineTiming::command_period_s() const { return axes_on_port * (write_s() + reads_per_step * read_s()); }

double LineTiming::min_step_s() const { return command_period_s() * margin; }

LineTiming LineTiming::model() const {
    LineTiming t = *this;
    t.measured_write_s.reset();
    t.measured_read_s.reset();
    return t;
}

Json LineTiming::to_json() const {
    Json j = Json::object();
    j.set("baudrate", baud_rate);
    j.set("axes_on_port", axes_on_port);
    j.set("response_ms", response_ms);
    j.set("gap_s", gap_s);
    j.set("turnaround_s", turnaround_s);
    j.set("reads_per_step", reads_per_step);
    j.set("margin", margin);
    j.set("measured_write_s", rounded(measured_write_s, 5));
    j.set("measured_read_s", rounded(measured_read_s, 5));
    j.set("write_s", round_to(write_s(), 5));
    j.set("read_s", round_to(read_s(), 5));
    j.set("command_period_s", round_to(command_period_s(), 5));
    j.set("min_step_s", round_to(min_step_s(), 5));
    return j;
}

bool LineTiming::operator==(const LineTiming& o) const {
    return baud_rate == o.baud_rate && axes_on_port == o.axes_on_port && response_ms == o.response_ms &&
           gap_s == o.gap_s && turnaround_s == o.turnaround_s && reads_per_step == o.reads_per_step &&
           margin == o.margin && measured_write_s == o.measured_write_s && measured_read_s == o.measured_read_s;
}

// ============================================================================================ MotionPlan

double MotionPlan::shortest_step_s() const {
    if (times.size() < 2) return 0.0;
    double best = kInf;
    for (std::size_t k = 0; k + 1 < times.size(); ++k) best = std::min(best, times[k + 1] - times[k]);
    return best;
}

Point MotionPlan::waypoint(int k) const {
    const int n = static_cast<int>(points.size());
    const int index = k < 0 ? n + k : k;
    if (index < 0 || index >= n) throw std::out_of_range("no waypoint " + std::to_string(k));
    Point p;
    for (std::size_t i = 0; i < axes.size(); ++i) p.set(axes[i], points[static_cast<std::size_t>(index)][i]);
    return p;
}

int MotionPlan::index_of(const std::string& axis) const {
    const auto it = std::find(axes.begin(), axes.end(), axis);
    if (it == axes.end()) throw std::invalid_argument("'" + axis + "' is not an axis of the plan");
    return static_cast<int>(it - axes.begin());
}

double MotionPlan::max_speed(const std::string& axis) const {
    const std::size_t i = static_cast<std::size_t>(index_of(axis));
    double best = 0.0;
    for (const auto& s : speeds) best = std::max(best, s[i]);
    return best;
}

double MotionPlan::max_accel(const std::string& axis) const {
    const std::size_t i = static_cast<std::size_t>(index_of(axis));
    double best = 0.0;
    for (const auto& a : accels) best = std::max(best, a[i]);
    return best;
}

bool MotionPlan::moves(const std::string& axis) const { return max_speed(axis) > 0; }

std::string MotionPlan::summary() const {
    const std::string units = ranges.empty() ? std::string("mm") : ranges.begin()->second.units;
    std::vector<std::string> lines;
    lines.push_back(std::string(to_string(mode)) + " plan over " + util::join(axes, ", ") + ": " +
                    std::to_string(steps()) + " step(s), " + f(length, 2) + " " + units + " in " + f(duration(), 2) +
                    " s at " + g6(path_speed) + " " + units + "/s (" + g6(accel_g) + " G ramps)");
    lines.push_back("shortest step " + f(shortest_step_s() * 1000, 1) + " ms (the line allows " +
                    f(min_step_s * 1000, 1) + " ms); chord error <= " + f(est_chord_error, 3) + "; timing error ~" +
                    f(est_timing_error_s * 1000, 1) + " ms; estimated deviation ~" + f(est_deviation, 3) + " " + units);
    if (lead_in)
        lines.push_back("lead-in first: a synchronized line from " + lead_in->start().to_string() + " (" +
                        f(lead_in->length, 2) + " " + units + ", " + f(lead_in->duration(), 2) + " s)");
    for (const auto& w : warnings) lines.push_back("warning: " + w);
    return util::join(lines, "\n");
}

Json MotionPlan::table() const {
    Json rows = Json::array();
    for (std::size_t k = 0; k < points.size(); ++k) {
        Json row = Json::object();
        row.set("k", static_cast<long long>(k));
        row.set("t", round_to(times[k], 4));
        for (std::size_t i = 0; i < axes.size(); ++i) row.set(axes[i], round_to(points[k][i], 3));
        if (k > 0)
            for (std::size_t i = 0; i < axes.size(); ++i) row.set("v_" + axes[i], speeds[k - 1][i]);
        rows.push_back(row);
    }
    return rows;
}

Json MotionPlan::to_json() const {
    auto matrix = [](const std::vector<std::vector<double>>& m, std::optional<int> decimals) {
        Json out = Json::array();
        for (const auto& row : m) {
            Json r = Json::array();
            for (double v : row) r.push_back(decimals ? round_to(v, *decimals) : v);
            out.push_back(r);
        }
        return out;
    };
    Json j = Json::object();
    j.set("mode", to_string(mode));
    j.set("axes", axes);
    j.set("points", matrix(points, 4));
    Json t = Json::array();
    for (double v : times) t.push_back(round_to(v, 5));
    j.set("times", t);
    j.set("speeds", matrix(speeds, std::nullopt));
    j.set("accels", matrix(accels, std::nullopt));
    j.set("path_speed", path_speed);
    j.set("accel_g", accel_g);
    j.set("chord_error", chord_error);
    j.set("length", round_to(length, 4));
    j.set("duration", round_to(duration(), 4));
    j.set("band", band);
    j.set("blend", blend);
    j.set("steps", steps());
    j.set("min_step_s", round_to(min_step_s, 5));
    j.set("shortest_step_s", round_to(shortest_step_s(), 5));
    j.set("est_chord_error", round_to(est_chord_error, 5));
    j.set("est_timing_error_s", round_to(est_timing_error_s, 5));
    j.set("est_deviation", round_to(est_deviation, 5));
    j.set("warnings", warnings);
    j.set("timing", timing ? timing->to_json() : Json());
    Json r = Json::object();
    for (const auto& kv : ranges) r.set(kv.first, kv.second.to_json());
    j.set("ranges", r);
    j.set("lead_in", lead_in ? lead_in->to_json() : Json());
    j.set("min_step_override", min_step_override);
    return j;
}

// ============================================================================================ plan()

MotionPlan plan(const Path& path, double speed, double accel_g, const PlanOptions& options) {
    speed = positive(speed, "speed");
    Common c;
    c.accel_g = positive(accel_g, "accel_g");
    c.chord_error = positive(options.chord_error, "chord_error");
    c.band = positive(options.band.value_or(0.1), "band");
    const std::vector<Piece> pieces = path.pieces(options.start);
    if (pieces.empty()) throw NoMotionError("the path does not move any axis");
    const std::vector<std::string> axes = pieces[0].start.axes();
    bool straight = true;
    for (const auto& p : pieces)
        if (!p.straight()) straight = false;
    Mode mode = options.mode;
    if (mode == Mode::Auto)
        mode = straight ? Mode::Sync : Mode::Stream;
    else if (mode == Mode::Sync && !straight)
        throw std::invalid_argument(
            "mode sync runs straight lines only; use mode stream (or auto) for arcs and circles");
    std::optional<double> blend;
    if (options.blend) {
        blend = positive(*options.blend, "blend");
        if (mode != Mode::Sync)
            throw std::invalid_argument("blend applies to mode sync (a streamed path never stops between its steps)");
    }
    std::optional<double> stream_accel_g;
    if (options.stream_accel_g) stream_accel_g = positive(*options.stream_accel_g, "stream_accel_g");
    for (const auto& a : axes) {
        const auto it = options.limits.find(a);
        c.ranges[a] = it == options.limits.end() ? AxisRange() : it->second;
    }
    c.timing = options.timing.value_or(LineTiming{});
    c.min_step = options.min_step_s ? std::max(0.0, number(*options.min_step_s, "min_step_s")) : c.timing.min_step_s();
    const std::function<MotionPlan(double)> attempt = [&](double v) {
        return mode == Mode::Sync ? plan_sync(pieces, axes, v, c, blend) : plan_stream(pieces, axes, v, c, stream_accel_g);
    };
    MotionPlan result;
    try {
        result = attempt(speed);
    } catch (const Infeasible& e) {
        const std::optional<double> best =
            e.checks.no_speed_helps ? std::nullopt : largest_speed(attempt, speed, e.checks.estimate());
        throw e.checks.error(best);
    }
    result.min_step_override = options.min_step_s.has_value();
    return result;
}

// ============================================================================================ results

std::pair<double, double> path_deviation(const std::vector<std::vector<double>>& points,
                                         const std::vector<std::vector<double>>& polyline) {
    if (points.empty() || polyline.empty()) return {0.0, 0.0};
    std::vector<std::pair<const std::vector<double>*, const std::vector<double>*>> segments;
    for (std::size_t k = 0; k + 1 < polyline.size(); ++k) segments.emplace_back(&polyline[k], &polyline[k + 1]);
    if (segments.empty()) segments.emplace_back(&polyline[0], &polyline[0]);
    double worst = 0.0, sum = 0.0;
    for (const auto& p : points) {
        double best = kInf;
        for (const auto& s : segments) best = std::min(best, segment_distance(p, *s.first, *s.second));
        worst = std::max(worst, best);
        sum += best;
    }
    return {worst, sum / static_cast<double>(points.size())};
}

std::string CoordinatedResult::summary() const {
    std::size_t total = 0;
    for (const auto& kv : samples) total += kv.second.size();
    const std::string state = completed ? std::string("completed") : "STOPPED: " + error.value_or("");
    return std::string(to_string(plan.mode)) + " run " + state + " in " + f(duration_s, 2) + " s (planned " +
           f(plan.duration(), 2) + " s): deviation max " + f(max_deviation, 3) + " / mean " + f(mean_deviation, 3) +
           " (estimated from " + std::to_string(trace.size()) + " trace points); " + std::to_string(frames) +
           " frames (" + std::to_string(skipped_frames) + " skipped), " + std::to_string(late_frames) + " late (worst " +
           f(worst_late_s * 1000, 1) + " ms), worst skew " + f(worst_skew_s * 1000, 1) + " ms; " +
           std::to_string(total) + " samples, ~" + f(sample_spacing_s * 1000, 0) + " ms apart per axis";
}

Json CoordinatedResult::to_json() const {
    Json j = Json::object();
    j.set("mode", to_string(plan.mode));
    j.set("axes", plan.axes);
    j.set("plan", plan.to_json());
    Json s = Json::object();
    for (const auto& a : plan.axes) {
        Json series = Json::array();
        const auto it = samples.find(a);
        if (it != samples.end())
            for (const auto& tp : it->second) {
                Json pair = Json::array();
                pair.push_back(round_to(tp.first, 4));
                pair.push_back(round_to(tp.second, 3));
                series.push_back(pair);
            }
        s.set(a, series);
    }
    j.set("samples", s);
    Json t = Json::array();
    for (const auto& row : trace) {
        Json r = Json::array();
        for (double v : row) r.push_back(round_to(v, 4));
        t.push_back(r);
    }
    j.set("trace", t);
    j.set("max_deviation", round_to(max_deviation, 4));
    j.set("mean_deviation", round_to(mean_deviation, 4));
    j.set("frames", frames);
    j.set("skipped_frames", skipped_frames);
    j.set("late_frames", late_frames);
    j.set("worst_late_s", round_to(worst_late_s, 5));
    j.set("worst_skew_s", round_to(worst_skew_s, 5));
    j.set("duration_s", round_to(duration_s, 4));
    j.set("completed", completed);
    j.set("error", error ? Json(*error) : Json());
    j.set("warnings", warnings);
    Json fin = Json::object();
    for (const auto& kv : final) fin.set(kv.first, round_to(kv.second, 3));
    j.set("final", fin);
    j.set("sample_spacing_s", round_to(sample_spacing_s, 5));
    return j;
}

namespace detail {

std::vector<std::vector<double>> merge_trace(
    const std::map<std::string, std::vector<std::pair<double, double>>>& samples, const std::vector<std::string>& axes,
    const std::vector<double>& start, double window) {
    std::map<std::string, std::vector<std::pair<double, double>>> series;
    std::vector<std::string> sampled;
    for (const auto& a : axes) {
        const auto it = samples.find(a);
        std::vector<std::pair<double, double>> data;
        if (it != samples.end()) data = it->second;
        std::sort(data.begin(), data.end());
        if (!data.empty()) sampled.push_back(a);
        series[a] = std::move(data);
    }
    if (sampled.empty()) return {};
    double lo = -kInf, hi = kInf;
    for (const auto& a : sampled) {
        lo = std::max(lo, series[a].front().first);
        hi = std::min(hi, series[a].back().first);
    }
    std::set<double> stamps;
    for (const auto& a : sampled)
        for (const auto& tp : series[a])
            if (lo <= tp.first && tp.first <= hi) stamps.insert(tp.first);
    std::vector<std::vector<double>> out;
    for (double t : stamps) {
        std::vector<double> row{t};
        bool keep = true;
        for (std::size_t i = 0; i < axes.size() && keep; ++i) {
            const auto& data = series[axes[i]];
            if (data.empty()) {
                row.push_back(start[i]);
                continue;
            }
            const auto it = std::lower_bound(data.begin(), data.end(), std::make_pair(t, -kInf));
            const std::size_t j = static_cast<std::size_t>(it - data.begin());
            double near = kInf;
            if (j >= 1) near = std::min(near, std::fabs(data[j - 1].first - t));
            if (j < data.size()) near = std::min(near, std::fabs(data[j].first - t));
            if (near > window) {
                keep = false;
                break;
            }
            if (j < data.size() && data[j].first == t)
                row.push_back(data[j].second);
            else if (j == 0)
                row.push_back(data[0].second);
            else if (j >= data.size())
                row.push_back(data.back().second);
            else {
                const auto& p0 = data[j - 1];
                const auto& p1 = data[j];
                row.push_back(p0.second + (p1.second - p0.second) * (t - p0.first) / (p1.first - p0.first));
            }
        }
        if (keep) out.push_back(row);
    }
    return out;
}

double sample_spacing(const std::map<std::string, std::vector<std::pair<double, double>>>& samples) {
    double worst = 0.0;
    for (const auto& kv : samples) {
        std::vector<double> stamps;
        for (const auto& tp : kv.second) stamps.push_back(tp.first);
        std::sort(stamps.begin(), stamps.end());
        if (stamps.size() < 2) continue;
        std::vector<double> gaps;
        for (std::size_t k = 0; k + 1 < stamps.size(); ++k) gaps.push_back(stamps[k + 1] - stamps[k]);
        worst = std::max(worst, median(gaps));
    }
    return worst;
}

void verify_plan(const MotionPlan& plan_, const std::map<std::string, AxisRange>& ranges) {
    Checks checks(plan_.path_speed);
    check_positions(checks, plan_.axes, plan_.points, ranges);
    check_axis_rates(checks, plan_.axes, plan_.speeds, plan_.accels, ranges, plan_.accel_g,
                     plan_.mode == Mode::Stream);
    if (plan_.mode == Mode::Stream) {  // sync: checked just above
        for (std::size_t i = 0; i < plan_.axes.size(); ++i) {
            const auto it = ranges.find(plan_.axes[i]);
            if (it == ranges.end() || !it->second.max_accel_g) continue;
            double top = 0.0;
            for (const auto& a : plan_.accels) top = std::max(top, a[i]);
            if (top > *it->second.max_accel_g + 1e-9)
                checks.add("axis " + plan_.axes[i] + ": frame acceleration " + g6(top) + " G above its maximum " +
                           g6(*it->second.max_accel_g) + " G");
        }
    }
    if (!checks.problems.empty()) throw checks.error(checks.estimate());
}

}  // namespace detail

}  // namespace coord
}  // namespace iai_actuator_driver
