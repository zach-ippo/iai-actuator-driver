// Coordinated motion: points, segments and paths (coord.hpp). Port of the Python edition's coord.py geometry (Line, Arc,
// Circle, Polyline, Path, _Piece, _arc_piece, _circle_piece). Pure: no I/O. UNVERIFIED on hardware in this edition.
#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "iai_actuator_driver/coord.hpp"
#include "util.hpp"

namespace iai_actuator_driver {
namespace coord {

namespace {

constexpr double kPi = 3.14159265358979323846;

std::string g6(double v) { return util::format("%g", v); }

/// Python's x % m for m > 0 (never negative).
double pymod(double x, double m) {
    double r = std::fmod(x, m);
    if (r < 0) r += m;
    return r;
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

/// A non-empty point with non-empty axis names and finite positions.
Point checked_point(const Point& value, const std::string& what) {
    if (value.empty())
        throw std::invalid_argument(what + " must be a non-empty {axis name: position}, got {}");
    for (const auto& kv : value) {
        if (kv.first.empty()) throw std::invalid_argument(what + ": axis names must be non-empty strings");
        number(kv.second, what + "['" + kv.first + "']");
    }
    return value;
}

Plane checked_plane(const Plane& plane) {
    if (plane.first.empty() || plane.second.empty() || plane.first == plane.second)
        throw std::invalid_argument("plane must be two different axis names, got (" + plane.first + ", " +
                                    plane.second + ")");
    return plane;
}

/// A point in the plane: {plane.first: .., plane.second: ..} (other axes ignored) or (a, b).
std::pair<double, double> plane_pair(const PlanePoint& value, const Plane& plane, const std::string& what) {
    if (value.point) {
        std::vector<std::string> missing;
        for (const auto& a : {plane.first, plane.second})
            if (!value.point->contains(a)) missing.push_back(a);
        if (!missing.empty())
            throw std::invalid_argument(what + " must give both plane axes (" + plane.first + ", " + plane.second +
                                        "), missing " + util::join(missing, ", "));
        return {number(value.point->at(plane.first), what), number(value.point->at(plane.second), what)};
    }
    if (!value.pair) throw std::invalid_argument(what + " must be (a, b) or {axis: position}");
    return {number(value.pair->first, what), number(value.pair->second, what)};
}

/// The axes of a Point outside the plane (held positions), {} for a pair.
Point others(const PlanePoint& value, const Plane& plane) {
    Point out;
    if (!value.point) return out;
    for (const auto& kv : checked_point(*value.point, "point"))
        if (kv.first != plane.first && kv.first != plane.second) out.set(kv.first, kv.second);
    return out;
}

bool close_to(const Point& a, const Point& b, double tol = 1e-9) {
    for (const auto& kv : a)
        if (std::fabs(kv.second - b.at(kv.first)) > tol) return false;
    return true;
}

Piece arc_piece(const Point& cur, const Arc& seg) {
    const std::string& a = seg.plane.first;
    const std::string& b = seg.plane.second;
    for (const auto& axis : {a, b})
        if (!cur.contains(axis))
            throw std::invalid_argument("no start position for axis '" + axis + "' of the arc's plane");
    const double cx = seg.center.first;
    const double cy = seg.center.second;
    const Point end = cur.merged(seg.end);
    const double r0 = std::hypot(cur.at(a) - cx, cur.at(b) - cy);
    const double r1 = std::hypot(end.at(a) - cx, end.at(b) - cy);
    if (r0 < 1e-6) throw std::invalid_argument("the arc starts at its center (" + g6(cx) + ", " + g6(cy) + ")");
    if (r1 < 1e-6) throw std::invalid_argument("the arc ends at its center (" + g6(cx) + ", " + g6(cy) + ")");
    if (std::min(r0, r1) < kMinRadius)
        throw std::invalid_argument("the arc's radius (" + g6(std::min(r0, r1)) + ") is below one position count (" +
                                    g6(kMinRadius) + ")");
    if (std::fabs(r1 - r0) > std::max(0.01, 0.005 * r0))
        throw std::invalid_argument("the arc's end is not on its circle: radius " + util::fixed(r0, 3) +
                                    " at the start, " + util::fixed(r1, 3) + " at the end");
    const double a0 = std::atan2(cur.at(b) - cy, cur.at(a) - cx);
    const double a1 = std::atan2(end.at(b) - cy, end.at(a) - cx);
    double sweep = 0.0;
    if (seg.ccw) {
        sweep = pymod(a1 - a0, 2 * kPi);
        if (sweep < 1e-9 || sweep > 2 * kPi - 1e-9) sweep = 2 * kPi;
    } else {
        sweep = -pymod(a0 - a1, 2 * kPi);
        if (sweep > -1e-9 || sweep < -2 * kPi + 1e-9) sweep = -2 * kPi;
    }
    Piece p;
    p.start = cur;
    p.end = end;
    p.plane = seg.plane;
    p.center = seg.center;
    p.r0 = r0;
    p.r1 = r1;
    p.a0 = a0;
    p.sweep = sweep;
    return p;
}

Piece circle_piece(const Point& cur, const Circle& seg) {
    const std::string& a = seg.plane.first;
    const std::string& b = seg.plane.second;
    for (const auto& axis : {a, b})
        if (!cur.contains(axis))
            throw std::invalid_argument("no start position for axis '" + axis + "' of the circle's plane");
    if (seg.radius < kMinRadius)
        throw std::invalid_argument("the circle's radius (" + g6(seg.radius) + ") is below one position count (" +
                                    g6(kMinRadius) + ")");
    const Point start = seg.start();
    const double tol = std::max(0.01, 0.005 * seg.radius);
    if (std::hypot(cur.at(a) - start.at(a), cur.at(b) - start.at(b)) > tol)
        throw std::invalid_argument("the circle starts at (" + util::fixed(start.at(a), 3) + ", " +
                                    util::fixed(start.at(b), 3) + ") but the path is at (" + util::fixed(cur.at(a), 3) +
                                    ", " + util::fixed(cur.at(b), 3) +
                                    "): add a Line to the circle's start first (Path::circle and "
                                    "CoordinatedGroup::circle do)");
    const double sweep = 2 * kPi * seg.turns * (seg.ccw ? 1.0 : -1.0);
    const bool whole = std::fabs(seg.turns - std::round(seg.turns)) < 1e-9;
    Piece p;
    p.start = cur;
    p.end = whole ? cur : cur.merged(seg.point_at(seg.start_deg + sweep * 180.0 / kPi));
    p.plane = seg.plane;
    p.center = seg.center;
    p.r0 = seg.radius;
    p.r1 = seg.radius;
    p.a0 = seg.start_deg * kPi / 180.0;
    p.sweep = sweep;
    return p;
}

void add_axes(std::vector<std::string>& names, const std::vector<std::string>& more) {
    for (const auto& n : more)
        if (std::find(names.begin(), names.end(), n) == names.end()) names.push_back(n);
}

}  // namespace

// ============================================================================================ Mode

const char* to_string(Mode mode) noexcept {
    switch (mode) {
        case Mode::Sync: return "sync";
        case Mode::Stream: return "stream";
        default: return "auto";
    }
}

Mode parse_mode(const std::string& name) {
    if (name == "auto") return Mode::Auto;
    if (name == "sync") return Mode::Sync;
    if (name == "stream") return Mode::Stream;
    throw std::invalid_argument("mode must be one of auto, sync, stream, got '" + name + "'");
}

// ============================================================================================ Point

Point::Point(std::initializer_list<value_type> values) {
    for (const auto& kv : values) set(kv.first, kv.second);
}

double& Point::operator[](const std::string& axis) {
    for (auto& kv : values_)
        if (kv.first == axis) return kv.second;
    values_.emplace_back(axis, 0.0);
    return values_.back().second;
}

double Point::at(const std::string& axis) const {
    for (const auto& kv : values_)
        if (kv.first == axis) return kv.second;
    throw std::out_of_range("no position for axis '" + axis + "' in " + to_string());
}

bool Point::contains(const std::string& axis) const {
    for (const auto& kv : values_)
        if (kv.first == axis) return true;
    return false;
}

void Point::set(const std::string& axis, double value) { (*this)[axis] = value; }

void Point::erase(const std::string& axis) {
    values_.erase(std::remove_if(values_.begin(), values_.end(), [&](const value_type& kv) { return kv.first == axis; }),
                  values_.end());
}

std::vector<std::string> Point::axes() const {
    std::vector<std::string> out;
    for (const auto& kv : values_) out.push_back(kv.first);
    return out;
}

Point Point::merged(const Point& other) const {
    Point out = *this;
    for (const auto& kv : other) out.set(kv.first, kv.second);
    return out;
}

bool Point::operator==(const Point& other) const {
    if (size() != other.size()) return false;
    for (const auto& kv : values_)
        if (!other.contains(kv.first) || other.at(kv.first) != kv.second) return false;
    return true;
}

std::string Point::to_string() const {
    std::string out = "{";
    for (std::size_t i = 0; i < values_.size(); ++i) {
        if (i) out += ", ";
        out += values_[i].first + ": " + g6(values_[i].second);
    }
    return out + "}";
}

Json Point::to_json() const {
    Json j = Json::object();
    for (const auto& kv : values_) j.set(kv.first, kv.second);
    return j;
}

// ============================================================================================ segments

Line::Line(Point to_) : to(checked_point(to_, "Line(to)")) {}
Line::Line(std::initializer_list<Point::value_type> to_) : Line(Point(to_)) {}

Arc::Arc(const PlanePoint& center_, Point end_, bool ccw_, Plane plane_)
    : center(plane_pair(center_, checked_plane(plane_), "Arc center")),
      end(checked_point(end_, "Arc end")),
      ccw(ccw_),
      plane(std::move(plane_)) {}

Circle::Circle(const PlanePoint& center_, double radius_, double start_deg_, double turns_, bool ccw_, Plane plane_)
    : center(plane_pair(center_, checked_plane(plane_), "Circle center")),
      radius(positive(radius_, "Circle radius")),
      start_deg(number(start_deg_, "Circle start_deg")),
      turns(positive(turns_, "Circle turns")),
      ccw(ccw_),
      plane(std::move(plane_)) {}

Point Circle::point_at(double deg) const {
    const double rad = deg * kPi / 180.0;
    Point p;
    p.set(plane.first, center.first + radius * std::cos(rad));
    p.set(plane.second, center.second + radius * std::sin(rad));
    return p;
}

Point Circle::start() const { return point_at(start_deg); }

Polyline::Polyline(std::vector<Point> points_) {
    for (std::size_t i = 0; i < points_.size(); ++i)
        points.push_back(checked_point(points_[i], "Polyline point " + std::to_string(i)));
    if (points.empty()) throw std::invalid_argument("Polyline needs at least one point");
}

// ============================================================================================ Piece

Point Piece::at(double f) const {
    if (f >= 1.0) return end;
    Point out;
    for (const auto& kv : start) out.set(kv.first, kv.second + (end.at(kv.first) - kv.second) * f);
    if (plane) {
        const double r = r0 + (r1 - r0) * f;
        const double angle = a0 + sweep * f;
        out.set(plane->first, center.first + r * std::cos(angle));
        out.set(plane->second, center.second + r * std::sin(angle));
    }
    return out;
}

double Piece::length() const {
    if (!plane) {
        double sum = 0.0;
        for (const auto& kv : start) {
            const double d = end.at(kv.first) - kv.second;
            sum += d * d;
        }
        return std::sqrt(sum);
    }
    double rest = 0.0;
    for (const auto& kv : start) {
        if (kv.first == plane->first || kv.first == plane->second) continue;
        const double d = end.at(kv.first) - kv.second;
        rest += d * d;
    }
    return std::hypot((r0 + r1) / 2 * std::fabs(sweep), std::sqrt(rest));
}

// ============================================================================================ Path

Path::Path(std::optional<Point> start, std::vector<Segment> segments) {
    if (start) start_ = checked_point(*start, "Path start");
    then(segments);
}

Path& Path::then(Segment segment) {
    segments_.push_back(std::move(segment));
    return *this;
}

Path& Path::then(const std::vector<Segment>& segments) {
    for (const auto& s : segments) segments_.push_back(s);
    return *this;
}

std::vector<std::string> Path::axes() const {
    std::vector<std::string> names;
    if (start_) add_axes(names, start_->axes());
    for (const auto& seg : segments_) {
        if (const auto* line = std::get_if<Line>(&seg)) {
            add_axes(names, line->to.axes());
        } else if (const auto* poly = std::get_if<Polyline>(&seg)) {
            for (const auto& p : poly->points) add_axes(names, p.axes());
        } else if (const auto* arc = std::get_if<Arc>(&seg)) {
            add_axes(names, {arc->plane.first, arc->plane.second});
            add_axes(names, arc->end.axes());
        } else if (const auto* circle = std::get_if<Circle>(&seg)) {
            add_axes(names, {circle->plane.first, circle->plane.second});
        }
    }
    return names;
}

bool Path::straight() const {
    for (const auto& seg : segments_)
        if (!std::holds_alternative<Line>(seg) && !std::holds_alternative<Polyline>(seg)) return false;
    return true;
}

std::vector<Piece> Path::pieces(const std::optional<Point>& start) const {
    std::optional<Point> origin = start ? std::optional<Point>(checked_point(*start, "start")) : start_;
    if (!origin)
        throw std::invalid_argument(
            "the path has no start point: give Path(start) or plan it from the axes' positions");
    std::vector<std::string> names = axes();
    add_axes(names, origin->axes());
    std::vector<std::string> missing;
    for (const auto& a : names)
        if (!origin->contains(a)) missing.push_back(a);
    if (!missing.empty())
        throw std::invalid_argument("no start position for axis/axes " + util::join(missing, ", "));
    Point cur;
    for (const auto& a : names) cur.set(a, origin->at(a));
    std::vector<Piece> out;
    auto line = [&](const Point& to) {
        const Point end = cur.merged(to);
        if (!close_to(cur, end)) {
            Piece p;
            p.start = cur;
            p.end = end;
            out.push_back(p);
        }
        cur = end;
    };
    for (const auto& seg : segments_) {
        if (const auto* l = std::get_if<Line>(&seg)) {
            line(l->to);
        } else if (const auto* poly = std::get_if<Polyline>(&seg)) {
            for (const auto& p : poly->points) line(p);
        } else if (const auto* arc = std::get_if<Arc>(&seg)) {
            out.push_back(arc_piece(cur, *arc));
            cur = out.back().end;
        } else if (const auto* circle = std::get_if<Circle>(&seg)) {
            out.push_back(circle_piece(cur, *circle));
            cur = out.back().end;
        }
    }
    return out;
}

double Path::length(const std::optional<Point>& start) const {
    double sum = 0.0;
    for (const auto& p : pieces(start)) sum += p.length();
    return sum;
}

std::string Path::to_string() const {
    std::ostringstream s;
    s << "Path(start=" << (start_ ? start_->to_string() : std::string("None")) << ", " << segments_.size()
      << " segment(s))";
    return s.str();
}

Path Path::circle(const PlanePoint& center, double radius, double start_deg, double turns, bool ccw, Plane plane) {
    const Circle seg(center, radius, start_deg, turns, ccw, plane);
    return Path(others(center, seg.plane).merged(seg.start()), {seg});
}

Path Path::rectangle(const PlanePoint& corner, double width, double height, Plane plane) {
    const Plane p = checked_plane(plane);
    const auto xy = plane_pair(corner, p, "rectangle corner");
    const double w = number(width, "width");
    const double h = number(height, "height");
    if (w == 0 || h == 0) throw std::invalid_argument("rectangle width and height must not be 0");
    const double x0 = xy.first;
    const double y0 = xy.second;
    const std::pair<double, double> corners[] = {{x0 + w, y0}, {x0 + w, y0 + h}, {x0, y0 + h}, {x0, y0}};
    std::vector<Point> points;
    for (const auto& c : corners) points.push_back(Point{{p.first, c.first}, {p.second, c.second}});
    Point start = others(corner, p);
    start.set(p.first, x0);
    start.set(p.second, y0);
    return Path(start, {Polyline(points)});
}

Path Path::polygon(const PlanePoint& center, double radius, int sides, double start_deg, Plane plane) {
    if (sides < 3) throw std::invalid_argument("sides must be an integer >= 3, got " + std::to_string(sides));
    const Circle ring(center, radius, start_deg, 1.0, true, plane);
    std::vector<Point> corners;
    for (int k = 1; k <= sides; ++k) corners.push_back(ring.point_at(start_deg + 360.0 * k / sides));
    corners.back() = ring.start();
    return Path(others(center, ring.plane).merged(ring.start()), {Polyline(corners)});
}

}  // namespace coord
}  // namespace iai_actuator_driver
