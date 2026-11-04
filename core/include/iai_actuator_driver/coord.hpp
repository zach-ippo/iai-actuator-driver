// Coordinated (interpolated) multi-axis motion, run by the PC: lines, arcs and circles over several axes. Port of the
// Python edition's coord.py (docs/DRIVER.md, "Coordinated motion (alpha)"); implemented by src/coord_geometry.cpp
// (points, segments, paths), src/coord_plan.cpp (the planner, line timing, plans and results) and src/coord_run.cpp
// (CoordinatedGroup, the runners, Cell::coordinated).
//
// IAI units have no interpolation of their own and no shared clock: every axis runs its own move. This module makes
// several axes follow one path from the PC side:
//
//   Cell cell = Cell::open(SimLine::sim_ports(2));
//   coord::CoordinatedGroup xy = cell.coordinated({{"x", "SIM1:1"}, {"y", "SIM2:1"}});
//   coord::MotionPlan plan = xy.plan(coord::Path::circle({75.0, 75.0}, 10.0), 20.0, 0.3);  // dry run, nothing sent
//   std::cout << plan.summary() << "\n";
//   coord::CoordinatedResult result = xy.run_plan(plan);   // or xy.circle({75.0, 75.0}, 10.0, 20.0)
//
// Coordinates are {axis name: position} (coord::Point) in each axis's own units (mm, or deg on a rotary axis); axes a
// segment does not name hold their position. Two ways of running a path:
// * Mode::Sync (straight lines, polylines): each segment is one move per axis, started on every port at once (ports in
//   parallel threads, the axes of a port back to back). Axis i gets speed v * |d_i| / L and acceleration =
//   deceleration a * |d_i| / L, so every axis runs the same trapezoid in time and the point stays on the straight
//   line. The next segment starts when every axis is done (blend: when every axis is within that distance).
// * Mode::Stream (arcs, circles, curves): the path is cut into chords (per arc the angle step is 2 acos(1 - e / r) for
//   the chord error e) and every waypoint gets a time from a trapezoid speed profile along the path length. At the
//   planned time t_k each axis is re-targeted to waypoint k + 1 (one step ahead) with the speed that gets it there on
//   time and an acceleration high enough to change speed within a quarter of the step (capped by the axis). One thread
//   per port runs the schedule against one start time (1 ms Windows timer); a frame whose target does not change is
//   skipped; the last waypoint completes by the axis's completion rule. A frame later than one full step aborts. A lost
//   echo is sent again at once (the frame is idempotent); both attempts lost are a link fault. Between frames each
//   axis's status is read about every kSampleGapS while the slack allows, and at least every kSampleBoundS even when
//   that makes a frame late, so a fault on one axis stops the others within a few steps.
// * Mode::Auto: sync for a path made only of lines, stream otherwise.
// A path the axes are not at the start of gets a straight lead-in from where they are.
//
// The planner (plan()) is pure and runs every check before anything is sent: every commanded waypoint inside each
// axis's range, every per-axis speed and acceleration within the axis's limits(), the centripetal acceleration v^2 / r
// within the smallest axis acceleration, and the step time not shorter than what the line can carry (LineTiming). An
// infeasible path throws InfeasiblePathError (a SoftLimitError) naming the reasons and the largest feasible speed
// (found by re-running the checks, bisection); nothing is sent. CoordinatedGroup::run_plan checks again against the
// axes as they are now.
//
// During a run the status monitors of the run's axes are held; other axes' monitors keep polling. Every transaction
// waits at most about three times its measured time, and the run never releases a STOP it did not send. Any error, a
// fault alarm (EMG, servo off, pause), a link fault, a frame later than one step, or an outside stop (group stop(),
// Cell::stop_all, the cell's stop on a link fault, Axis::stop / pause / servo OFF from another thread, from the moment
// run_plan is called) stops every axis of the group (decel stop) and throws CoordinatedMotionError (the result so far
// in result()). A link fault in a cell goes to the cell's fault handling once the group's STOP frames are out (the
// cell's own link check, Cell::check_links, confirms it and follows its stop_on_fault policy). One run at a time per
// axis.
//
// Evidence: a 0x9900 block re-sent mid-move re-targets the move and a new speed takes effect mid-move (VERIFIED on the
// EC gripper in the Python edition); there, with one real axis (the EC-GRBP8M gripper) and a simulated second axis, a
// sync line, streamed circles and the outside stops ran on hardware (VERIFIED). Two or more real axes together are
// UNVERIFIED. This C++ port is UNVERIFIED on hardware (simulator only). The PC has no shared clock with the units:
// frame timing sets the smoothness.
#pragma once

#include <cstdint>
#include <exception>
#include <functional>
#include <initializer_list>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/device_info.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/status.hpp"

namespace iai_actuator_driver {
namespace coord {

// ------------------------------------------------------------------------------------------ constants
inline constexpr double kG = reg::kGmms2;         ///< 1 G in units/s^2 (IAI: 9800 mm/s^2)
inline constexpr double kLineStepS = 0.1;         ///< stream: a straight piece is cut into steps of at most this long
inline constexpr double kMaxChordAngle = 1.5707963267948966;  ///< at most a quarter turn per chord
inline constexpr double kMinRadius = 0.01;        ///< an arc or circle radius below one position count is refused
inline constexpr double kStartLeadS = 0.05;       ///< the schedule starts this long after the port threads start
inline constexpr double kJitterS = 0.002;         ///< PC timing jitter assumed by the estimates (1 ms timer)
inline constexpr double kLateFraction = 0.1;      ///< a frame later than this share of its step (and 2 ms) is late
inline constexpr int kSilentReads = 3;            ///< failed status reads in a row on one axis = a link fault
inline constexpr double kPollS = 0.005;           ///< status poll gap while waiting for the last waypoint
inline constexpr double kSampleBoundS = 0.15;     ///< stream: an axis unread this long is read even if a frame goes late
inline constexpr double kSampleGapS = 0.02;       ///< stream: while the slack allows, each axis is read about this often
inline constexpr double kWaitFactor = 3.0;        ///< a run's transaction waits at most this x its measured time ...
inline constexpr double kWaitMinS = 0.05;         ///< ... and at least this long
inline constexpr double kWatchS = 0.01;           ///< the main thread looks at the ports' errors this often
inline constexpr int kSlowBaud = 38400;           ///< below this rate every reply is taken as slow as ...
inline constexpr double kSlowReplyS = 0.285;      ///< ... the worst seen at 9600 (VERIFIED on the EC gripper)
inline constexpr int kWakeRounds = 3;             ///< automatic servo OFF checks before a segment / a stream
inline constexpr double kMeasureTolerance = 0.8;  ///< run_plan refuses a plan below this share of the new minimum step
inline constexpr int kMeasureSamples = 3;         ///< the line measurement: this many transactions per kind and axis
inline constexpr int kWriteRequestBytes = 27;     ///< FC10, 9 words at 0x9900: request 27 bytes, echo 8
inline constexpr int kWriteResponseBytes = 8;
inline constexpr int kReadRequestBytes = 8;       ///< FC03, 8 words at 0x9000 (the lean status): request 8, answer 21
inline constexpr int kReadResponseBytes = 21;

/// How a path runs.
enum class Mode {
    Auto,    ///< Sync for a path made only of lines, Stream otherwise
    Sync,    ///< one move per axis and segment, proportional speeds and accelerations
    Stream,  ///< time-scheduled re-targeting one waypoint ahead
};
/// "auto", "sync", "stream".
const char* to_string(Mode mode) noexcept;
/// The mode of a name; std::invalid_argument otherwise.
Mode parse_mode(const std::string& name);

// ------------------------------------------------------------------------------------------ geometry

/// {axis name: position}, in insertion order (the order of a plan's axes follows it). Assigning an axis again keeps
/// its place.
class Point {
public:
    using value_type = std::pair<std::string, double>;
    using const_iterator = std::vector<value_type>::const_iterator;

    Point() = default;
    Point(std::initializer_list<value_type> values);  // NOLINT: implicit on purpose ({{"x", 1.0}, {"y", 2.0}})

    double& operator[](const std::string& axis);       ///< inserts 0 at the end when absent
    double at(const std::string& axis) const;          ///< std::out_of_range when absent
    bool contains(const std::string& axis) const;
    void set(const std::string& axis, double value);
    void erase(const std::string& axis);
    std::size_t size() const { return values_.size(); }
    bool empty() const { return values_.empty(); }
    std::vector<std::string> axes() const;
    const_iterator begin() const { return values_.begin(); }
    const_iterator end() const { return values_.end(); }
    /// This point with `other`'s axes set over it (Python {**self, **other}).
    Point merged(const Point& other) const;
    /// The same axes with the same values (order ignored).
    bool operator==(const Point& other) const;
    bool operator!=(const Point& other) const { return !(*this == other); }
    /// "{x: 10, y: 20.5}".
    std::string to_string() const;
    Json to_json() const;

private:
    std::vector<value_type> values_;
};

/// Two different axis names: the plane of an arc or circle (plane.first to the right, plane.second up).
using Plane = std::pair<std::string, std::string>;

/// A point in a plane: (a, b), or a Point naming both plane axes (its other axes: positions held by a demo shape).
struct PlanePoint {
    PlanePoint(double a, double b) : pair(std::make_pair(a, b)) {}
    PlanePoint(std::initializer_list<Point::value_type> values) : point(Point(values)) {}  // NOLINT
    PlanePoint(const Point& values) : point(values) {}  // NOLINT: implicit on purpose
    std::optional<std::pair<double, double>> pair;
    std::optional<Point> point;
};

/// A straight move to `to`; axes not named hold their position.
struct Line {
    Point to;
    explicit Line(Point to);
    Line(std::initializer_list<Point::value_type> to);  // NOLINT: Line({{"x", 40.0}})
};

/// A circular arc in `plane` from the current point around `center` to `end` (ccw: counter-clockwise seen with
/// plane.first to the right and plane.second up). `end` == the start is a full circle. Axes of `end` outside the plane
/// move linearly along the arc (a helix); the others hold. The end must lie on the circle through the start (0.5 % of
/// the radius, at least 0.01).
struct Arc {
    std::pair<double, double> center;
    Point end;
    bool ccw = true;
    Plane plane;
    Arc(const PlanePoint& center, Point end, bool ccw = true, Plane plane = {"x", "y"});
};

/// `turns` full turns (fractions allowed) in `plane` around `center` with `radius`, starting and (for whole turns)
/// ending at the angle start_deg. The path must already be at that start point (Path::circle and
/// CoordinatedGroup::circle put it there).
struct Circle {
    std::pair<double, double> center;
    double radius = 0.0;
    double start_deg = 0.0;
    double turns = 1.0;
    bool ccw = true;
    Plane plane;
    Circle(const PlanePoint& center, double radius, double start_deg = 0.0, double turns = 1.0, bool ccw = true,
           Plane plane = {"x", "y"});
    /// The point of the circle at `deg` (plane axes only).
    Point point_at(double deg) const;
    Point start() const;
};

/// Straight moves through `points` one after another.
struct Polyline {
    std::vector<Point> points;
    explicit Polyline(std::vector<Point> points);
};

using Segment = std::variant<Line, Arc, Circle, Polyline>;

/// One resolved piece of a path: straight (no plane) or an arc; both carry every axis of the path.
struct Piece {
    Point start;
    Point end;
    std::optional<Plane> plane;
    std::pair<double, double> center{0.0, 0.0};
    double r0 = 0.0;
    double r1 = 0.0;
    double a0 = 0.0;
    double sweep = 0.0;  ///< signed, radians

    bool straight() const { return !plane; }
    /// The point at fraction f (0..1) of the piece.
    Point at(double f) const;
    double length() const;
};

/// A start point plus segments. No start: the path starts wherever the axes are (CoordinatedGroup::plan reads them).
/// then() appends segments and returns the path.
class Path {
public:
    explicit Path(std::optional<Point> start = std::nullopt, std::vector<Segment> segments = {});

    Path& then(Segment segment);
    Path& then(const std::vector<Segment>& segments);
    const std::optional<Point>& start() const { return start_; }
    const std::vector<Segment>& segments() const { return segments_; }
    /// Every axis the path names, start first, in order of appearance.
    std::vector<std::string> axes() const;
    /// True when the path is made only of lines (Mode::Auto runs it synchronized).
    bool straight() const;
    /// The resolved pieces from `start` (default: the path's own), every piece carrying every axis. Zero-length lines are
    /// dropped. std::invalid_argument for an axis without a start position, an arc whose end is off its circle, or a
    /// Circle the path is not at the start of.
    std::vector<Piece> pieces(const std::optional<Point>& start = std::nullopt) const;
    /// The exact path length (arcs as arcs), in the axes' units.
    double length(const std::optional<Point>& start = std::nullopt) const;
    std::string to_string() const;

    // ---- demo shapes
    /// A path that starts on the circle at start_deg and runs `turns` around it. Axes of a `center` Point outside the
    /// plane are held at those positions.
    static Path circle(const PlanePoint& center, double radius, double start_deg = 0.0, double turns = 1.0,
                       bool ccw = true, Plane plane = {"x", "y"});
    /// A closed rectangle from `corner` (width along plane.first, height along plane.second; negative values go the other
    /// way), four lines back to the corner.
    static Path rectangle(const PlanePoint& corner, double width, double height, Plane plane = {"x", "y"});
    /// A closed regular polygon with `sides` corners on the circle (center, radius), the first corner at start_deg.
    static Path polygon(const PlanePoint& center, double radius, int sides, double start_deg = 0.0,
                        Plane plane = {"x", "y"});

private:
    std::optional<Point> start_;
    std::vector<Segment> segments_;
};

// ------------------------------------------------------------------------------------------ limits and line timing

/// The ranges the planner checks one axis against: position (min, max), max_speed (units/s), max_accel_g (one value
/// for accel and decel, as a 0x9900 move uses), min_speed (the documented minimum: a warning, never a refusal).
/// nullopt = not checked.
struct AxisRange {
    std::optional<Span> position;
    std::optional<double> max_speed;
    std::optional<double> max_accel_g;
    std::optional<double> min_speed;
    std::string units = "mm";

    /// From Axis::limits(): position, speed max and recommended_min, accel_decel max, units.
    static AxisRange of(const AxisRanges& limits);
    /// A range by its values.
    static AxisRange make(double min, double max, std::optional<double> max_speed = std::nullopt,
                          std::optional<double> max_accel_g = std::nullopt,
                          std::optional<double> min_speed = std::nullopt);
    Json to_json() const;
    bool operator==(const AxisRange& o) const;
};

/// How often one port can re-target its axes: the estimate behind the planner's shortest step (the model is short of
/// the EC gripper's measured 82 / 53 ms at 38400 / 230400 baud, VERIFIED in the Python edition: the measured times
/// count). Per transaction the model takes the bytes on the wire (10 bits each), the unit's response delay RTIM (EC
/// item 8.4 / CON No.17, 5 ms by default, DOC; below 38400 baud at least the ~285 ms reply VERIFIED at 9600), the bus's
/// minimum gap and an allowance for the USB adapter and the PC (turnaround_s). measured_write_s / measured_read_s: what
/// one transaction took on this line just before (a full status read, longer on the wire than the 0x9900 write, and a
/// lean status read; per axis the median of kMeasureSamples, the slowest axis). Per step and axis one 0x9900 write
/// plus reads_per_step lean status reads; the axes of a port take turns; margin covers the rest. LineTiming{38400},
/// LineTiming{115200, 3}: a rate and the axes on the port, the rest by default.
struct LineTiming {
    LineTiming(int baud_rate_ = 115200, int axes_on_port_ = 1)  // NOLINT: implicit on purpose
        : baud_rate(baud_rate_), axes_on_port(axes_on_port_) {}

    int baud_rate = 115200;
    int axes_on_port = 1;
    double response_ms = 5.0;
    double gap_s = 0.005;
    double turnaround_s = 0.002;
    int reads_per_step = 1;
    double margin = 1.5;
    std::optional<double> measured_write_s;
    std::optional<double> measured_read_s;

    /// One transaction by the model (no measurement), seconds.
    double frame_s(int request_bytes, int response_bytes) const;
    double write_s() const;
    double read_s() const;
    /// One step of the port: every axis written once and read reads_per_step times.
    double command_period_s() const;
    double min_step_s() const;
    /// The same line by the model alone (the measurements dropped).
    LineTiming model() const;
    Json to_json() const;
    bool operator==(const LineTiming& o) const;
    bool operator!=(const LineTiming& o) const { return !(*this == o); }
};

// ------------------------------------------------------------------------------------------ plans

/// A planned coordinated move (no I/O). points[k] are the waypoints (points[0] the start) in `axes` order, times[k]
/// their planned times (s), speeds[k] / accels[k] what each axis is commanded for step k (points[k] -> points[k + 1];
/// units/s and G, 0 = no frame: the axis's target does not change). Sync: one step per segment; stream: one step per
/// chord. lead_in: a synchronized line from where the axes were to points[0], run first. min_step_override: min_step_s
/// was given (the run then does not check the steps against the line again).
struct MotionPlan {
    Mode mode = Mode::Sync;
    std::vector<std::string> axes;
    std::vector<std::vector<double>> points;
    std::vector<double> times;
    std::vector<std::vector<double>> speeds;
    std::vector<std::vector<double>> accels;
    double path_speed = 0.0;
    double accel_g = 0.0;
    double chord_error = 0.0;
    double length = 0.0;
    double band = 0.1;
    double min_step_s = 0.0;
    double est_chord_error = 0.0;
    double est_timing_error_s = 0.0;
    double est_deviation = 0.0;
    std::optional<double> blend;
    std::vector<std::string> warnings;
    std::optional<LineTiming> timing;
    std::map<std::string, AxisRange> ranges;
    std::shared_ptr<MotionPlan> lead_in;
    bool min_step_override = false;

    int steps() const { return static_cast<int>(points.size()) - 1; }
    double duration() const { return times.empty() ? 0.0 : times.back(); }
    Point start() const { return waypoint(0); }
    Point end() const { return waypoint(-1); }
    double shortest_step_s() const;
    /// Waypoint k (negative: from the end).
    Point waypoint(int k) const;
    int index_of(const std::string& axis) const;  ///< std::invalid_argument when not an axis of the plan
    double max_speed(const std::string& axis) const;
    double max_accel(const std::string& axis) const;
    /// True when `axis` gets at least one frame.
    bool moves(const std::string& axis) const;
    std::string summary() const;
    /// One row per waypoint: k, t, the positions and (for the step that ends there) the axis speeds v_<axis>.
    Json table() const;
    Json to_json() const;
};

/// Settings of plan() and CoordinatedGroup::plan / run.
struct PlanOptions {
    double chord_error = 0.05;             ///< the largest distance between an arc and its chords (stream)
    Mode mode = Mode::Auto;
    /// {axis: range} (axes missing there are not range-checked). Ignored by CoordinatedGroup (it reads the axes').
    std::map<std::string, AxisRange> limits;
    /// The shortest step the line can carry (default: timing.min_step_s()); given, run_plan does not check the steps
    /// against the line again.
    std::optional<double> min_step_s;
    /// The line (default LineTiming{}). Ignored by CoordinatedGroup (it measures its own).
    std::optional<LineTiming> timing;
    std::optional<double> band;            ///< the positioning band of every frame (default 0.1; a group: its band)
    std::optional<Point> start;            ///< overrides the path's start (ignored by CoordinatedGroup)
    std::optional<double> blend;           ///< sync: the next segment starts once every axis is within this distance
    std::optional<double> stream_accel_g;  ///< a fixed re-targeting acceleration instead of the automatic one
};

/// A path refused by the planner (or by run_plan's checks): nothing was sent. reasons() lists every refusal,
/// max_speed() the largest speed that passes every check (found by re-running them), nullopt when no speed helps.
class InfeasiblePathError : public SoftLimitError {
public:
    InfeasiblePathError(const std::string& message, std::vector<std::string> reasons, std::optional<double> max_speed)
        : SoftLimitError(message), reasons_(std::move(reasons)), max_speed_(max_speed) {}
    const std::vector<std::string>& reasons() const noexcept { return reasons_; }
    const std::optional<double>& max_speed() const noexcept { return max_speed_; }

private:
    std::vector<std::string> reasons_;
    std::optional<double> max_speed_;
};

/// The path moves no axis (by at least one position count) from its start.
class NoMotionError : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
};

/// Plan `path` at path speed `speed` (units/s) with ramps of accel_g (G). Pure: no I/O. Throws InfeasiblePathError
/// when the path is infeasible, NoMotionError for a path that moves no axis, std::invalid_argument for a bad argument
/// (Mode::Sync with an arc, a blend outside Mode::Sync, a non-positive value).
MotionPlan plan(const Path& path, double speed, double accel_g = 0.1, const PlanOptions& options = {});

/// (max, mean) perpendicular distance of `points` from `polyline`. (0, 0) for no points.
std::pair<double, double> path_deviation(const std::vector<std::vector<double>>& points,
                                         const std::vector<std::vector<double>>& polyline);

// ------------------------------------------------------------------------------------------ results

/// What a coordinated run did: the plan, the status samples per axis ((t, position), t from the planned start), the
/// merged trace ((t, p...) in plan axis order, only where every axis was read within one read time), the deviation of
/// the trace from the planned polyline (an estimate from the status samples: blind between samples; sample_spacing_s is
/// the median time between two samples of one axis), the frames sent / skipped, late frames (stream: on the wire more
/// than 10 % of their step after the planned time) and the worst lateness, the worst start skew between the axes of
/// one step, the duration.
struct CoordinatedResult {
    MotionPlan plan;
    std::map<std::string, std::vector<std::pair<double, double>>> samples;
    std::vector<std::vector<double>> trace;
    double max_deviation = 0.0;
    double mean_deviation = 0.0;
    int frames = 0;
    int skipped_frames = 0;
    int late_frames = 0;
    double worst_late_s = 0.0;
    double worst_skew_s = 0.0;
    double duration_s = 0.0;
    bool completed = false;
    std::optional<std::string> error;
    std::vector<std::string> warnings;
    Point final;  ///< the last sampled position of every axis sampled
    double sample_spacing_s = 0.0;

    std::string summary() const;
    Json to_json() const;
};

/// A coordinated run failed or was stopped from outside. Every axis of the group was sent a decel stop first.
/// result() is the result so far, axis() the axis that failed (nullopt: not one axis), stop() what the group stop
/// returned per axis, cause() the error behind it.
class CoordinatedMotionError : public EcDriverError {
public:
    CoordinatedMotionError(const std::string& message, CoordinatedResult result, std::optional<std::string> axis,
                           CellResults<Status> stop, std::exception_ptr cause)
        : EcDriverError(message), result_(std::make_shared<CoordinatedResult>(std::move(result))),
          axis_(std::move(axis)), stop_(std::make_shared<CellResults<Status>>(std::move(stop))), cause_(cause) {}
    const CoordinatedResult& result() const noexcept { return *result_; }
    const std::optional<std::string>& axis() const noexcept { return axis_; }
    const CellResults<Status>& stop() const noexcept { return *stop_; }
    std::exception_ptr cause() const noexcept { return cause_; }

private:
    std::shared_ptr<CoordinatedResult> result_;
    std::optional<std::string> axis_;
    std::shared_ptr<CellResults<Status>> stop_;
    std::exception_ptr cause_;
};

// ------------------------------------------------------------------------------------------ the group

/// Settings of CoordinatedGroup::run / run_plan.
struct RunOptions {
    std::optional<double> late_abort_s;  ///< stream: abort when a frame reaches the wire later than this (default: its step)
    /// For the last waypoint (default: from the plan). A value must be finite and above 0: 0, a negative value ("wait
    /// forever" in some languages) or an infinity is refused by run / run_plan / line_to / circle with SoftLimitError
    /// before anything is sent (the C ABI's request: IAD_ERR_INVALID_ARGUMENT). nullopt (absent) is the default.
    std::optional<double> timeout_s;
    /// run() (line_to, circle): where the plan it makes must start ({name: position}; a binding's run_plan sends the
    /// start of the plan it holds): each named axis within max(band, 0.05) of that plan's start (its lead-in's, if
    /// any), else EcNotReadyError and nothing is sent. Checked after planning, inside the run's stop window, so a stop
    /// sent meanwhile still ends the run. nullopt (absent): no such check. run_plan, given the plan itself, ignores it:
    /// it checks the axes against that plan's start.
    std::optional<Point> expect_start;
};

/// Settings of CoordinatedGroup::circle.
struct CircleOptions {
    std::optional<Plane> plane;          ///< default: the group's first two axes
    /// nullopt: start where the axes are when they are on the circle (within 0.5 % of the radius, at least 0.01; no
    /// lead-in), else at 0 deg after a straight lead-in.
    std::optional<double> start_deg;
    double turns = 1.0;
    bool ccw = true;
};

/// Several axes ({name, axis}) moved along one path. Build it directly or with Cell::coordinated. Path coordinates use
/// these names. The axes may sit on one port or several (grouped by their bus: ports run in parallel, the axes of a
/// port take turns). plan() is the dry run; run() / run_plan() move; line_to() and circle() are shortcuts from the
/// current position; stop() decel-stops every axis of the group (and ends a run in another thread). A cheap handle:
/// copies share the group. Thread-safe as the Python edition's: one run at a time, stop() from any thread.
class CoordinatedGroup {
public:
    using AxisList = std::vector<std::pair<std::string, std::shared_ptr<Axis>>>;

    /// A group of axes outside a cell. std::invalid_argument when empty, a name or an axis appears twice.
    explicit CoordinatedGroup(AxisList axes, double band_mm = 0.1);
    /// A group of cell axes (Cell::coordinated): `cell_names` maps each name to its cell axis name. The cell must
    /// outlive the group and stay where it is.
    CoordinatedGroup(AxisList axes, Cell* cell, std::map<std::string, std::string> cell_names, double band_mm = 0.1);

    std::vector<std::string> names() const;
    Axis& axis(const std::string& name) const;              ///< std::out_of_range for a name not in the group
    std::shared_ptr<Axis> axis_ptr(const std::string& name) const;
    double band() const;
    Cell* cell() const;
    std::optional<CoordinatedResult> last_result() const;
    std::string to_string() const;

    // ---- reading (no motion)
    /// Every axis's AxisRange from Axis::limits(), read now (a stroke narrowed since counts). Throws when an axis's
    /// soft limits cannot be read and no stroke is configured (as a move would).
    std::map<std::string, AxisRange> ranges();
    /// The slowest port's LineTiming (baud rate, response delay, gap, the group's axes on it). `measure`: also time
    /// full and lean status reads of every axis (ports in parallel); the frame and read times are then at least those.
    LineTiming timing(const std::vector<std::string>& names = {}, bool measure = false);
    /// The current positions (one status read per axis).
    Point positions(const std::vector<std::string>& names = {});

    // ---- planning
    /// Dry run: plan `path` from the axes' current positions against their own ranges and the line's timing (measured
    /// now). Nothing is sent (reads only). A path with a start the axes are not at gets a straight lead-in from where
    /// they are (a warning says so): one more segment of a sync path, MotionPlan::lead_in (run first, synchronized) for
    /// a streamed one. NoMotionError when the path does not move any axis from where they are.
    MotionPlan plan(const Path& path, double speed, double accel_g = 0.1, const PlanOptions& options = {});

    // ---- running
    /// plan() and run_plan(). A path that moves no axis from where they are (e.g. line_to the current position)
    /// returns a completed result at once, nothing sent - only when every axis is at the path's end. An outside stop
    /// counts from the call, the planning included.
    CoordinatedResult run(const Path& path, double speed, double accel_g = 0.1, const PlanOptions& plan_options = {},
                          const RunOptions& run_options = {});
    /// Run a plan (its lead_in first). Before anything is sent every check runs again against the axes as they are now:
    /// ranges read fresh, every waypoint inside each axis's current fence, every axis of the plan ready (held ones too)
    /// and at the start, a streamed plan's steps against the line timing measured now. On any failure, a link fault or
    /// an outside stop every axis of the group is stopped and CoordinatedMotionError thrown. An axis in another
    /// coordinated run is refused (EcNotReadyError, nothing sent).
    CoordinatedResult run_plan(const MotionPlan& plan, const RunOptions& options = {});
    /// A synchronized straight move from the current position to `point` (axes not named hold). At `point` already: a
    /// completed result, nothing sent.
    CoordinatedResult line_to(const Point& point, double speed, double accel_g = 0.1,
                              const PlanOptions& plan_options = {}, const RunOptions& run_options = {});
    /// A streamed circle (see CircleOptions).
    CoordinatedResult circle(const PlanePoint& center, double radius, double speed, double accel_g = 0.1,
                             const CircleOptions& circle = {}, const PlanOptions& plan_options = {},
                             const RunOptions& run_options = {});
    /// Decel stop on every axis of the group, as fast as possible: the STOP frame to every axis first (ports in
    /// parallel), then each axis holds it until it stands still and releases it. In a cell this is Cell::stop_all on
    /// the group's axes. A run of this group in another thread sends nothing more and throws CoordinatedMotionError.
    /// Never throws for one axis: the failures are in the result's errors.
    CellResults<Status> stop(double timeout_s = 5.0);

    struct State;  ///< private (src/coord_run.cpp)

private:
    std::shared_ptr<State> s_;
};

// ------------------------------------------------------------------------------------------ internals (tests)
namespace detail {

/// The status samples merged into (t, p_0, p_1 ...) rows at every sample time where every sampled axis has a sample of
/// its own within `window`; each axis interpolated linearly between its own samples (CoordinatedResult::trace).
std::vector<std::vector<double>> merge_trace(
    const std::map<std::string, std::vector<std::pair<double, double>>>& samples, const std::vector<std::string>& axes,
    const std::vector<double>& start, double window);

/// The median time between two samples of one axis, the worst axis (0 with fewer than two samples).
double sample_spacing(const std::map<std::string, std::vector<std::pair<double, double>>>& samples);

/// A plan against `ranges` read now (it may have been made elsewhere, or a stroke narrowed since): positions, per-axis
/// speeds and accelerations, a streamed plan's frame accelerations. InfeasiblePathError; no I/O.
void verify_plan(const MotionPlan& plan, const std::map<std::string, AxisRange>& ranges);

/// Process-wide test hooks of the coordinated runs (the Python tests patch the module instead). Tests only: never set
/// in an application. Set them before a run and clear them after it.
struct CoordTestHooks {
    /// Called by run() between its plan and run_plan (on the caller's thread).
    std::function<void(const MotionPlan& plan)> after_plan;
    /// Called by run_plan before each plan it runs (the lead-in, then the path), before that plan's checks.
    std::function<void(const MotionPlan& plan)> before_execute;
    /// Called in the port thread just before each attempt of a 0x9900 frame of `axis`.
    std::function<void(const std::string& axis)> before_frame;
    /// Called in a port thread as it records the error that ended that port, before the run sees it; a throw stands
    /// for a failure while recording it (the run must still end in an error).
    std::function<void()> on_port_error;
    /// Called in a first-STOP thread right after the first STOP frame of `axis` went out (or failed).
    std::function<void(const std::string& axis)> after_stop_frame;
    /// Stream: every frame's wake-up this much later than planned (a busy PC).
    double schedule_delay_s = 0.0;
};
CoordTestHooks& coord_test_hooks();

}  // namespace detail
}  // namespace coord

// The names of the Python edition's top level (iai_actuator_driver.LineSegment is coord.Line there).
using LineSegment = coord::Line;
using coord::CoordinatedGroup;
using coord::CoordinatedMotionError;
using coord::CoordinatedResult;

}  // namespace iai_actuator_driver
