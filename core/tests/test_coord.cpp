// Coordinated motion, pure parts: geometry (lines, arcs, circles, demo shapes), the planner's checks (ranges, per-axis
// speed and acceleration, centripetal acceleration, the line timing), the largest feasible speed, step merging, the
// deviation helpers. Port of the Python edition's tests/test_coord.py (GeometryTest, PlannerTest). No I/O.
#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "iai_actuator_driver/coord.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::coord;

namespace {

constexpr double kPi = 3.14159265358979323846;

AxisRange range_xy() { return AxisRange::make(0.0, 150.0, 700.0, 1.0); }

std::map<std::string, AxisRange> limits_xy() { return {{"x", range_xy()}, {"y", range_xy()}}; }

PlanOptions with_limits(std::optional<double> min_step = std::nullopt) {
    PlanOptions o;
    o.limits = limits_xy();
    o.min_step_s = min_step;
    return o;
}

/// The what() of the exception `fn` throws ("" when none).
std::string error_of(const std::function<void()>& fn) {
    try {
        fn();
    } catch (const std::exception& e) {
        return e.what();
    }
    return "";
}

bool contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

std::vector<std::vector<double>> pts(std::initializer_list<std::vector<double>> rows) { return rows; }

}  // namespace

// ============================================================================================ geometry

TEST(coord_geometry_arc_and_circle_lengths) {
    const Path quarter(Point{{"x", 10.0}, {"y", 0.0}}, {Arc({0, 0}, {{"x", 0.0}, {"y", 10.0}})});
    CHECK_NEAR(quarter.length(), 5 * kPi, 1e-9);
    const Path cw(Point{{"x", 10.0}, {"y", 0.0}}, {Arc({0, 0}, {{"x", 0.0}, {"y", 10.0}}, false)});
    CHECK_NEAR(cw.length(), 15 * kPi, 1e-9);
    CHECK_NEAR(Path::circle({0, 0}, 10.0).length(), 20 * kPi, 1e-9);
    CHECK_NEAR(Path::circle({0, 0}, 10.0, 0.0, 2.5).length(), 50 * kPi, 1e-9);
    const Path helix(Point{{"x", 10.0}, {"y", 0.0}, {"z", 0.0}},
                     {Arc({0, 0}, {{"x", 10.0}, {"y", 0.0}, {"z", 5.0}})});
    CHECK_NEAR(helix.length(), std::hypot(20 * kPi, 5.0), 1e-9);
}

TEST(coord_geometry_rectangle_polygon_and_circle_points) {
    const MotionPlan rect = plan(Path::rectangle({10, 20}, 30, 15), 20, 0.1, with_limits());
    CHECK(rect.mode == Mode::Sync);
    CHECK(rect.points == pts({{10, 20}, {40, 20}, {40, 35}, {10, 35}, {10, 20}}));
    const Path hexagon = Path::polygon({{"x", 50.0}, {"y", 50.0}, {"z", 7.0}}, 10.0, 6);
    REQUIRE(hexagon.start().has_value());
    CHECK(*hexagon.start() == (Point{{"z", 7.0}, {"x", 60.0}, {"y", 50.0}}));
    const auto pieces = hexagon.pieces();
    CHECK_EQ(pieces.size(), std::size_t(6));
    for (const auto& p : pieces) {
        CHECK_NEAR(std::hypot(p.end.at("x") - 50, p.end.at("y") - 50), 10.0, 1e-9);
        CHECK_EQ(p.end.at("z"), 7.0);
    }
    const MotionPlan circle = plan(Path::circle({75, 75}, 10.0, 90), 20, 0.3, with_limits());
    CHECK(circle.points.front() == circle.points.back());
    CHECK_NEAR(circle.points[0][0], 75.0, 1e-9);
    CHECK_NEAR(circle.points[0][1], 85.0, 1e-9);
    for (const auto& p : circle.points) CHECK_NEAR(std::hypot(p[0] - 75, p[1] - 75), 10.0, 1e-9);
    CHECK(circle.points[1][0] < 75.0);  // counter-clockwise from 90 deg: x decreases first
}

TEST(coord_geometry_chord_count_follows_the_chord_error) {
    const std::pair<double, double> cases[] = {{10.0, 0.05}, {10.0, 0.01}, {50.0, 0.05}, {2.0, 0.2}};
    for (const auto& c : cases) {
        const double r = c.first, e = c.second;
        PlanOptions o = with_limits(0.0);
        o.chord_error = e;
        const MotionPlan p = plan(Path::circle({75, 75}, r), 10, 0.3, o);
        const double step = 2 * std::acos(1 - e / r);
        CHECK_EQ(p.steps(), std::max(4, static_cast<int>(std::ceil(2 * kPi / step - 1e-9))));
        CHECK(p.est_chord_error <= e + 1e-12);
        double worst = -1.0;
        for (std::size_t k = 0; k + 1 < p.points.size(); ++k) {
            const double mx = (p.points[k][0] + p.points[k + 1][0]) / 2;
            const double my = (p.points[k][1] + p.points[k + 1][1]) / 2;
            worst = std::max(worst, r - std::hypot(mx - 75, my - 75));
        }
        CHECK_NEAR(worst, p.est_chord_error, 1e-9);
    }
}

TEST(coord_geometry_bad_geometry_is_refused) {
    CHECK_THROWS(Path(Point{{"x", 10.0}, {"y", 0.0}}, {Arc({0, 0}, {{"x", 0.0}, {"y", 12.0}})}).pieces(),
                 std::invalid_argument);
    CHECK_THROWS(Path(Point{{"x", 0.0}, {"y", 0.0}}, {Circle({50, 50}, 5.0)}).pieces(), std::invalid_argument);
    CHECK_THROWS(Path(std::nullopt, {LineSegment({{"x", 1.0}})}).pieces(), std::invalid_argument);
    CHECK_THROWS(Path(Point{{"x", 0.0}}, {LineSegment({{"y", 1.0}})}).pieces(), std::invalid_argument);  // y has no start
    CHECK_THROWS(Circle({0, 0}, -1.0), std::invalid_argument);
    CHECK_THROWS(Path::rectangle({0, 0}, 0, 5), std::invalid_argument);
    CHECK_THROWS(plan(Path(Point{{"x", 1.0}}, {LineSegment({{"x", 1.0}})}), 10, 0.1), NoMotionError);  // does not move
    PlanOptions sync;
    sync.mode = Mode::Sync;
    CHECK_THROWS(plan(Path::circle({75, 75}, 5.0), 10, 0.1, sync), std::invalid_argument);
    CHECK_THROWS(LineSegment(Point()), std::invalid_argument);
    CHECK_THROWS(Arc({0, 0}, {{"x", 1.0}}, true, {"x", "x"}), std::invalid_argument);
    CHECK_THROWS(Polyline({}), std::invalid_argument);
    CHECK_THROWS(Path::polygon({0, 0}, 5.0, 2), std::invalid_argument);
    CHECK_THROWS(parse_mode("fast"), std::invalid_argument);
    CHECK(parse_mode("stream") == Mode::Stream);
}

TEST(coord_geometry_degenerate_arcs_throw) {
    // an arc from 5 microns off its center to the center passed the end tolerance and divided by zero
    PlanOptions o;
    o.min_step_s = 0.0;
    std::string e = error_of([&] {
        plan(Path(Point{{"x", 75.005}, {"y", 75.0}}, {Arc({75, 75}, {{"x", 75.0}, {"y", 75.0}})}), 10, 0.3, o);
    });
    CHECK(contains(e, "ends at its center"));
    e = error_of([&] {
        plan(Path(Point{{"x", 75.005}, {"y", 75.0}}, {Arc({75, 75}, {{"x", 74.995}, {"y", 75.0}})}), 10, 0.3, o);
    });
    CHECK(contains(e, "below one position count"));
    e = error_of([&] { plan(Path::circle({75, 75}, 0.005), 10, 0.3, o); });
    CHECK(contains(e, "below one position count"));
}

TEST(coord_geometry_axes_not_named_hold_and_polyline_equals_lines) {
    const Path path(Point{{"x", 0.0}, {"y", 0.0}, {"z", 3.0}},
                    {Polyline({Point{{"x", 10.0}}, Point{{"y", 5.0}}}), LineSegment({{"z", 4.0}})});
    CHECK(path.straight());
    const MotionPlan p = plan(path, 10, 0.1);
    CHECK(p.axes == (std::vector<std::string>{"x", "y", "z"}));
    CHECK(p.points == pts({{0, 0, 3}, {10, 0, 3}, {10, 5, 3}, {10, 5, 4}}));
    CHECK(p.speeds[0] == (std::vector<double>{10.0, 0.0, 0.0}));  // only x moves in the first segment
    // Path::axes: start first, then in order of appearance; then() appends
    Path more(Point{{"b", 1.0}});
    more.then(LineSegment({{"a", 2.0}})).then(Arc({0, 0}, {{"x", 1.0}, {"y", 0.0}}));
    CHECK(more.axes() == (std::vector<std::string>{"b", "a", "x", "y"}));
    CHECK(!more.straight());
}

// ============================================================================================ planner

TEST(coord_plan_sync_speed_and_accel_scale_with_each_axis_share) {
    const MotionPlan p = plan(Path(Point{{"x", 10.0}, {"y", 10.0}}, {LineSegment({{"x", 40.0}, {"y", 20.0}})}), 50, 0.3,
                              with_limits());
    const double length = std::hypot(30, 10);
    auto r2 = [](double v) { return std::round(v * 100) / 100; };
    CHECK(p.speeds[0] == (std::vector<double>{r2(50 * 30 / length), r2(50 * 10 / length)}));
    CHECK(p.accels[0] == (std::vector<double>{r2(0.3 * 30 / length), r2(0.3 * 10 / length)}));
    const double a = 0.3 * reg::kGmms2;
    CHECK_NEAR(p.duration(), length / 50 + 50 / a, 1e-9);  // trapezoid: L / v + v / a
    CHECK(contains(p.summary(), "sync plan over x, y"));
    CHECK(!p.to_json().dump().empty());
    CHECK_EQ(p.table().size(), std::size_t(2));
}

TEST(coord_plan_stream_profile_is_a_trapezoid_along_the_path) {
    PlanOptions o = with_limits();
    o.mode = Mode::Stream;
    const MotionPlan p = plan(Path(Point{{"x", 0.0}, {"y", 0.0}}, {LineSegment({{"x", 100.0}})}), 50, 0.01, o);
    CHECK(p.mode == Mode::Stream);
    const double a = 0.01 * reg::kGmms2;
    CHECK_NEAR(p.duration(), 100.0 / 50 + 50 / a, 1e-6);
    std::vector<double> speeds;
    for (std::size_t k = 0; k + 1 < p.points.size(); ++k)
        speeds.push_back(std::hypot(p.points[k + 1][0] - p.points[k][0], p.points[k + 1][1] - p.points[k][1]) /
                         (p.times[k + 1] - p.times[k]));
    CHECK(speeds.front() < 50);
    CHECK(speeds.back() < 50);
    CHECK_NEAR(*std::max_element(speeds.begin(), speeds.end()), 50, 0.5);
    CHECK(p.shortest_step_s() >= p.min_step_s - 1e-9);  // the short ramp steps merged (collinear)
}

TEST(coord_plan_auto_mode) {
    CHECK(plan(Path::rectangle({10, 10}, 5, 5), 10, 0.1).mode == Mode::Sync);
    PlanOptions o;
    o.min_step_s = 0.0;
    CHECK(plan(Path::circle({75, 75}, 5.0), 10, 0.1, o).mode == Mode::Stream);
}

TEST(coord_plan_soft_limit_refusal) {
    try {
        plan(Path::circle({145, 75}, 10.0), 20, 0.3, with_limits());
        CHECK(false);
    } catch (const InfeasiblePathError& e) {
        CHECK(contains(e.what(), "soft limits"));
        CHECK(!e.max_speed().has_value());
        CHECK(contains(e.what(), "no speed"));
        CHECK(!e.reasons().empty());
    }
}

TEST(coord_plan_the_start_point_itself_is_not_range_checked) {
    // the axes already stand at points[0] (e.g. -0.01 inside the unit's soft limit, outside the effective stroke):
    // only the commanded waypoints count, as for a single-axis move
    const MotionPlan p = plan(Path(Point{{"x", -0.01}, {"y", 5.0}}, {LineSegment({{"x", 5.0}, {"y", 5.0}})}), 10, 0.1,
                              with_limits());
    CHECK(p.points.back() == (std::vector<double>{5.0, 5.0}));
    const std::string e = error_of([] {
        plan(Path(Point{{"x", 5.0}, {"y", 5.0}}, {LineSegment({{"x", -0.02}})}), 10, 0.1, with_limits());
    });
    CHECK(contains(e, "waypoint 1"));
}

TEST(coord_plan_axis_speed_refusal_names_the_largest_feasible_speed) {
    PlanOptions slow;
    slow.limits = {{"x", range_xy()}, {"y", AxisRange::make(0.0, 150.0, 10.0, 1.0)}};
    const Path path(Point{{"x", 10.0}, {"y", 10.0}}, {LineSegment({{"x", 40.0}, {"y", 40.0}})});
    try {
        plan(path, 50, 0.3, slow);
        CHECK(false);
    } catch (const InfeasiblePathError& e) {
        CHECK(contains(e.what(), "axis y"));
        REQUIRE(e.max_speed().has_value());
        CHECK_NEAR(*e.max_speed(), 10 * std::sqrt(2.0), 0.02);
        const MotionPlan ok = plan(path, *e.max_speed(), 0.3, slow);
        CHECK(ok.max_speed("y") <= 10.0);
    }
}

TEST(coord_plan_accel_refusals) {
    PlanOptions gentle;
    gentle.limits = {{"x", AxisRange::make(0.0, 150.0, 700.0, 0.2)}, {"y", range_xy()}};
    CHECK(contains(error_of([&] {
                       plan(Path(Point{{"x", 10.0}, {"y", 10.0}}, {LineSegment({{"x", 40.0}})}), 50, 0.3, gentle);
                   }),
                   "acceleration"));
    PlanOptions o;  // centripetal: v^2 / r above the smallest axis accel
    o.limits = {{"x", AxisRange::make(0.0, 150.0, 700.0, 0.1)}, {"y", AxisRange::make(0.0, 150.0, 700.0, 0.1)}};
    o.min_step_s = 0.0;
    o.chord_error = 0.2;
    try {
        plan(Path::circle({75, 75}, 1.0), 100, 0.05, o);
        CHECK(false);
    } catch (const InfeasiblePathError& e) {
        CHECK(contains(e.what(), "centripetal"));
        REQUIRE(e.max_speed().has_value());
        CHECK_NEAR(*e.max_speed(), std::sqrt(0.1 * reg::kGmms2 * 1.0), 0.6);
    }
}

TEST(coord_plan_step_period_refusal) {
    try {
        plan(Path::circle({75, 75}, 10.0), 100, 0.3, with_limits(0.2));
        CHECK(false);
    } catch (const InfeasiblePathError& e) {
        CHECK(contains(e.what(), "the line can carry"));
        REQUIRE(e.max_speed().has_value());
        CHECK(*e.max_speed() > 0);
        CHECK(plan(Path::circle({75, 75}, 10.0), *e.max_speed(), 0.3, with_limits(0.2)).shortest_step_s() >=
              0.2 - 1e-9);
    }
    // the default estimate: slower at 38400 than at 230400, more axes per port take longer
    CHECK(LineTiming{38400}.min_step_s() > LineTiming{230400}.min_step_s());
    CHECK_NEAR((LineTiming{115200, 3}.command_period_s()), 3 * LineTiming{}.command_period_s(), 1e-12);
    // below 38400 every reply is taken as slow as the worst one seen at 9600 (~285 ms), and a measurement counts
    CHECK(LineTiming{9600}.min_step_s() > 0.8);
    LineTiming measured;
    measured.measured_read_s = 0.04;
    measured.measured_write_s = 0.05;
    CHECK_NEAR(measured.command_period_s(), 0.09, 1e-12);
    CHECK(measured.model() == LineTiming{});
    CHECK(!measured.to_json().dump().empty());
}

TEST(coord_plan_max_speed_passes_when_the_profile_never_reaches_cruise) {
    struct Case {
        Path path;
        double speed;
        double accel;
        std::map<std::string, AxisRange> limits;
        std::optional<double> min_step;
    };
    AxisRange position_only;
    position_only.position = Span{0, 150};
    AxisRange slow_x = AxisRange::make(0, 150, 5.0);
    const std::vector<Case> cases = {
        {Path::circle({75, 75}, 10.0), 500, 0.01, {{"x", position_only}, {"y", position_only}}, 0.2},
        {Path::circle({75, 75}, 10.0), 500, 0.01, {{"x", slow_x}, {"y", range_xy()}}, 0.0},
        {Path::circle({75, 75}, 10.0), 500, 0.1, limits_xy(), std::nullopt},
        {Path(Point{{"x", 85.15}, {"y", 75.0}}, {LineSegment({{"x", 85.0}}), Circle({75, 75}, 10.0)}), 20, 0.3, limits_xy(),
         std::nullopt},
    };
    for (const auto& c : cases) {
        PlanOptions o;
        o.limits = c.limits;
        o.min_step_s = c.min_step;
        std::optional<double> best;
        try {
            plan(c.path, c.speed, c.accel, o);
            CHECK(false);
        } catch (const InfeasiblePathError& e) {
            best = e.max_speed();
        }
        REQUIRE(best.has_value());
        CHECK(error_of([&] { plan(c.path, *best, c.accel, o); }).empty());       // passes at the advertised speed
        CHECK(!error_of([&] { plan(c.path, *best + 0.05, c.accel, o); }).empty());  // and it is about the largest
    }
}

TEST(coord_plan_frameless_and_tiny_steps_merge) {
    // a sub-count tail and a float arc end followed by its rounded value: no step without a frame, none too short
    const double ex = 75 + 10 * std::cos(1.0), ey = 75 + 10 * std::sin(1.0);
    auto r3 = [](double v) { return std::round(v * 1000) / 1000; };
    Path tail = Path::circle({75, 75}, 10);
    tail.then(LineSegment({{"x", 85.0004}}));
    const std::vector<Path> paths = {
        tail,
        Path(Point{{"x", 85.0}, {"y", 75.0}},
             {Arc({75, 75}, {{"x", ex}, {"y", ey}}), LineSegment({{"x", r3(ex)}, {"y", r3(ey)}}),
              LineSegment({{"x", 60.0}, {"y", 70.0}})}),
    };
    for (const auto& path : paths) {
        const MotionPlan p = plan(path, 20, 0.3, with_limits());
        CHECK(p.shortest_step_s() >= p.min_step_s - 1e-9);
        for (const auto& sp : p.speeds) {
            bool any = false;
            for (double v : sp) any = any || v > 0;
            CHECK(any);
        }
        CHECK(p.est_chord_error <= 0.05 + 1e-9);
    }
}

TEST(coord_plan_min_speed_is_one_warning_per_axis) {
    PlanOptions o;
    o.limits = {{"x", AxisRange::make(0, 150, 700.0, 1.0, 12.5)}, {"y", AxisRange::make(0, 150, 700.0, 1.0, 12.5)}};
    const MotionPlan p = plan(Path::circle({75, 75}, 10.0), 20, 0.3, o);
    int n = 0;
    for (const auto& w : p.warnings)
        if (contains(w, "documented minimum")) ++n;
    CHECK_EQ(n, 2);
}

TEST(coord_plan_axis_range_from_limits) {
    AxisRanges r;
    r.units = "mm";
    r.position.min = 0.0;
    r.position.max = 150.0;
    r.speed.max = 700.0;
    r.speed.recommended_min = 12.5;
    r.accel_decel.max = 1.0;
    r.accel.max = 2.0;
    CHECK(AxisRange::of(r) == AxisRange::make(0.0, 150.0, 700.0, 1.0, 12.5));
    CHECK(!AxisRange::of(r).to_json().dump().empty());
}

TEST(coord_plan_deviation_helper_and_merge_window) {
    const std::vector<std::vector<double>> line = {{0.0, 0.0}, {10.0, 0.0}};
    const auto dev = path_deviation({{5.0, 1.0}, {5.0, -3.0}}, line);
    CHECK_NEAR(dev.first, 3.0, 1e-12);
    CHECK_NEAR(dev.second, 2.0, 1e-12);
    CHECK_NEAR(path_deviation({{12.0, 0.0}}, line).first, 2.0, 1e-12);
    CHECK(path_deviation({}, line) == std::make_pair(0.0, 0.0));
    // y read far from x's sample at 0.05: that point is left out rather than interpolated across 90 ms
    const std::map<std::string, std::vector<std::pair<double, double>>> samples = {
        {"x", {{0.0, 0.0}, {0.05, 5.0}, {0.1, 10.0}}}, {"y", {{0.0, 0.0}, {0.001, 0.0}, {0.1, 0.0}}}};
    const auto rows = coord::detail::merge_trace(samples, {"x", "y"}, {0.0, 0.0}, 0.005);
    std::vector<double> stamps;
    for (const auto& row : rows) stamps.push_back(row[0]);
    CHECK(stamps == (std::vector<double>{0.0, 0.001, 0.1}));
    CHECK_NEAR(rows[1][1], 0.1, 1e-9);  // x interpolated between its own samples
    CHECK_NEAR(coord::detail::sample_spacing(samples), 0.05, 1e-12);
}

TEST(coord_plan_point_keeps_insertion_order) {
    Point p{{"y", 1.0}, {"x", 2.0}};
    p["y"] = 3.0;
    p.set("z", 4.0);
    CHECK(p.axes() == (std::vector<std::string>{"y", "x", "z"}));
    CHECK_EQ(p.at("y"), 3.0);
    CHECK(p == (Point{{"z", 4.0}, {"x", 2.0}, {"y", 3.0}}));
    CHECK_THROWS(p.at("w"), std::out_of_range);
    CHECK(p.merged(Point{{"x", 9.0}}).at("x") == 9.0);
    CHECK_EQ(p.to_string(), std::string("{y: 3, x: 2, z: 4}"));
}
