// C ABI 0.2: coordinated motion of several cell axes (include/iai_actuator_driver.h, "cell: coordinated motion"; the
// model is in docs/dev/NATIVE_ABI.md). The request is one JSON object read here into a coord::Path and the plan and run
// options; the work is CoordinatedGroup::plan / run (src/coord_run.cpp) on a group of the cell's axes made for the call
// (Cell::coordinated: a cheap handle). The errors are mapped in c_api_support.cpp (CoordinatedMotionError,
// InfeasiblePathError, NoMotionError).
//
// Evidence: UNVERIFIED on hardware (tested on the simulator: tests/test_c_api_ext.cpp).

#include <cmath>
#include <initializer_list>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "c_api_impl.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/coord.hpp"
#include "util.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::capi;

namespace {

// ------------------------------------------------------------------------------------------ request reading

[[noreturn]] void bad(const std::string& where, const std::string& what) {
    throw std::invalid_argument("request: " + where + " " + what);
}

const Json& object_of(const Json& j, const std::string& where) {
    if (j.type() != Json::Type::Object) bad(where, "must be a JSON object");
    return j;
}

double number_of(const Json& j, const std::string& where) {
    if (j.type() != Json::Type::Int && j.type() != Json::Type::Number) bad(where, "must be a number");
    return j.as_number();
}

bool bool_of(const Json& j, const std::string& where) {
    if (j.type() != Json::Type::Bool) bad(where, "must be true or false");
    return j.as_bool();
}

const std::string& string_of(const Json& j, const std::string& where) {
    if (j.type() != Json::Type::String) bad(where, "must be a string");
    return j.as_string();
}

/// Refuse a key of `j` that is not in `allowed` (a typo must not be ignored silently).
void only(const Json& j, std::initializer_list<const char*> allowed, const std::string& where) {
    for (const auto& kv : j.as_object()) {
        bool known = false;
        for (const char* a : allowed) known = known || kv.first == a;
        if (!known) {
            std::string list;
            for (const char* a : allowed) list += std::string(list.empty() ? "" : ", ") + a;
            bad(where, "has an unknown key '" + kv.first + "' (known: " + list + ")");
        }
    }
}

std::string at(const std::string& where, const char* key) { return where + "." + key; }

/// The value of `key` (nullptr when absent or null).
const Json* member(const Json& j, const char* key) {
    const Json* v = j.find(key);
    return v && !v->is_null() ? v : nullptr;
}

const Json& required(const Json& j, const char* key, const std::string& where) {
    const Json* v = member(j, key);
    if (!v) bad(at(where, key), "is required");
    return *v;
}

std::optional<double> opt_number(const Json& j, const char* key, const std::string& where) {
    const Json* v = member(j, key);
    if (!v) return std::nullopt;
    return number_of(*v, at(where, key));
}

double number_or(const Json& j, const char* key, double fallback, const std::string& where) {
    return opt_number(j, key, where).value_or(fallback);
}

/// The "timeout_s" of a request (RunOptions::timeout_s, the wait for the last waypoint): absent or null = the default
/// from the plan, else a finite number of seconds above 0. 0 or a negative value ("wait forever" in VB and LabVIEW) is
/// refused here, before anything is sent (a run would end its wait before it began and decel-stop at once). JSON has no
/// NaN or infinity; a number past the double range is refused when the text is parsed.
std::optional<double> timeout_of(const Json& j, const std::string& where) {
    const std::optional<double> timeout = opt_number(j, "timeout_s", where);
    if (timeout && !(std::isfinite(*timeout) && *timeout > 0.0))
        bad("timeout_s", "must be a finite number of seconds > 0 (absent or null: the default), got " +
                             util::g(*timeout) + "; nothing sent");
    return timeout;
}

bool bool_or(const Json& j, const char* key, bool fallback, const std::string& where) {
    const Json* v = member(j, key);
    return v ? bool_of(*v, at(where, key)) : fallback;
}

/// {"x": 10, "y": 20}.
coord::Point point_of(const Json& j, const std::string& where) {
    object_of(j, where);
    if (j.size() == 0) bad(where, "names no axis");
    coord::Point p;
    for (const auto& kv : j.as_object()) p.set(kv.first, number_of(kv.second, where + "." + kv.first));
    return p;
}

/// ["x", "y"]; absent: {"x", "y"}.
coord::Plane plane_of(const Json& j, const std::string& where) {
    const Json* v = member(j, "plane");
    if (!v) return {"x", "y"};
    const std::string w = at(where, "plane");
    if (v->type() != Json::Type::Array || v->size() != 2) bad(w, "must be two axis names, e.g. [\"x\", \"y\"]");
    return {string_of(v->as_array()[0], w + "[0]"), string_of(v->as_array()[1], w + "[1]")};
}

/// [a, b] in the plane, or a point naming both plane axes.
coord::PlanePoint plane_point_of(const Json& j, const std::string& where) {
    if (j.type() == Json::Type::Array) {
        if (j.size() != 2) bad(where, "must be [a, b] or a point");
        return coord::PlanePoint(number_of(j.as_array()[0], where + "[0]"), number_of(j.as_array()[1], where + "[1]"));
    }
    return coord::PlanePoint(point_of(j, where));
}

coord::Segment segment_of(const Json& j, const std::string& where) {
    object_of(j, where);
    if (j.size() != 1) bad(where, "must have one key: line, polyline, arc or circle");
    const std::string& kind = j.as_object()[0].first;
    const Json& v = j.as_object()[0].second;
    const std::string w = where + "." + kind;
    if (kind == "line") return coord::Line(point_of(v, w));
    if (kind == "polyline") {
        if (v.type() != Json::Type::Array || v.size() == 0) bad(w, "must be a list of points");
        std::vector<coord::Point> points;
        for (std::size_t i = 0; i < v.size(); ++i) points.push_back(point_of(v.as_array()[i], w + "[" + std::to_string(i) + "]"));
        return coord::Polyline(std::move(points));
    }
    if (kind == "arc") {
        object_of(v, w);
        only(v, {"center", "end", "ccw", "plane"}, w);
        return coord::Arc(plane_point_of(required(v, "center", w), at(w, "center")),
                          point_of(required(v, "end", w), at(w, "end")), bool_or(v, "ccw", true, w), plane_of(v, w));
    }
    if (kind == "circle") {
        object_of(v, w);
        only(v, {"center", "radius", "start_deg", "turns", "ccw", "plane"}, w);
        return coord::Circle(plane_point_of(required(v, "center", w), at(w, "center")),
                             number_of(required(v, "radius", w), at(w, "radius")), number_or(v, "start_deg", 0.0, w),
                             number_or(v, "turns", 1.0, w), bool_or(v, "ccw", true, w), plane_of(v, w));
    }
    bad(where, "has an unknown segment '" + kind + "' (known: line, polyline, arc, circle)");
}

coord::Path path_of(const Json& j, const std::string& where) {
    object_of(j, where);
    if (const Json* shape = member(j, "shape")) {
        const std::string& s = string_of(*shape, at(where, "shape"));
        if (s == "circle") {
            only(j, {"shape", "center", "radius", "start_deg", "turns", "ccw", "plane"}, where);
            return coord::Path::circle(plane_point_of(required(j, "center", where), at(where, "center")),
                                       number_of(required(j, "radius", where), at(where, "radius")),
                                       number_or(j, "start_deg", 0.0, where), number_or(j, "turns", 1.0, where),
                                       bool_or(j, "ccw", true, where), plane_of(j, where));
        }
        if (s == "rectangle") {
            only(j, {"shape", "corner", "width", "height", "plane"}, where);
            return coord::Path::rectangle(plane_point_of(required(j, "corner", where), at(where, "corner")),
                                          number_of(required(j, "width", where), at(where, "width")),
                                          number_of(required(j, "height", where), at(where, "height")),
                                          plane_of(j, where));
        }
        if (s == "polygon") {
            only(j, {"shape", "center", "radius", "sides", "start_deg", "plane"}, where);
            const double sides = number_of(required(j, "sides", where), at(where, "sides"));
            if (!(sides >= 3.0 && sides <= 100000.0) || sides != static_cast<double>(static_cast<int>(sides)))
                bad(at(where, "sides"), "must be an integer from 3 to 100000");
            return coord::Path::polygon(plane_point_of(required(j, "center", where), at(where, "center")),
                                        number_of(required(j, "radius", where), at(where, "radius")),
                                        static_cast<int>(sides), number_or(j, "start_deg", 0.0, where),
                                        plane_of(j, where));
        }
        bad(at(where, "shape"), "must be circle, rectangle or polygon, got '" + s + "'");
    }
    only(j, {"start", "segments"}, where);
    std::optional<coord::Point> start;
    if (const Json* s = member(j, "start")) start = point_of(*s, at(where, "start"));
    const Json& segments = required(j, "segments", where);
    if (segments.type() != Json::Type::Array || segments.size() == 0)
        bad(at(where, "segments"), "must be a non-empty list");
    std::vector<coord::Segment> list;
    for (std::size_t i = 0; i < segments.size(); ++i)
        list.push_back(segment_of(segments.as_array()[i], at(where, "segments") + "[" + std::to_string(i) + "]"));
    return coord::Path(std::move(start), std::move(list));
}

/// A parsed request: the group's axes, the path and its options.
struct Request {
    std::vector<std::pair<std::string, std::string>> axes;  ///< {path name, cell axis}; empty: every axis
    coord::Path path;
    double speed = 0.0;
    double accel_g = 0.1;
    coord::PlanOptions plan;
    coord::RunOptions run;
};

Request parse_request(const char* text) {
    need(text, "request");
    const Json j = Json::parse(text);
    const std::string w = "request";
    if (j.type() != Json::Type::Object) throw std::invalid_argument("request: must be a JSON object");
    only(j, {"axes", "band", "path", "speed", "accel_g", "mode", "chord_error", "min_step_s", "blend", "stream_accel_g",
             "late_abort_s", "timeout_s", "expect_start"},
         "the request");
    Request r;
    if (const Json* axes = member(j, "axes")) {
        if (axes->type() == Json::Type::Object) {
            for (const auto& kv : axes->as_object()) r.axes.emplace_back(kv.first, string_of(kv.second, "axes." + kv.first));
        } else if (axes->type() == Json::Type::Array) {
            for (std::size_t i = 0; i < axes->size(); ++i) {
                const std::string& name = string_of(axes->as_array()[i], "axes[" + std::to_string(i) + "]");
                r.axes.emplace_back(name, name);
            }
        } else {
            bad("axes", "must be {\"x\": \"COM3:1\", ...} or [\"COM3:1\", ...]");
        }
        if (r.axes.empty()) bad("axes", "names no axis (leave it out for every axis of the cell)");
    }
    r.path = path_of(required(j, "path", w), "path");
    r.speed = number_of(required(j, "speed", w), "speed");
    r.accel_g = number_or(j, "accel_g", 0.1, w);
    if (const Json* mode = member(j, "mode")) r.plan.mode = coord::parse_mode(string_of(*mode, "mode"));
    r.plan.chord_error = number_or(j, "chord_error", r.plan.chord_error, w);
    r.plan.band = opt_number(j, "band", w);
    r.plan.min_step_s = opt_number(j, "min_step_s", w);
    r.plan.blend = opt_number(j, "blend", w);
    r.plan.stream_accel_g = opt_number(j, "stream_accel_g", w);
    r.run.late_abort_s = opt_number(j, "late_abort_s", w);
    r.run.timeout_s = timeout_of(j, w);
    if (const Json* start = member(j, "expect_start")) r.run.expect_start = point_of(*start, at(w, "expect_start"));
    return r;
}

/// The JSON of a plan or a result with its summary line.
template <class T>
Json with_summary(const T& value) {
    Json j = value.to_json();
    j.set("summary", value.summary());
    return j;
}

void keep(CellEntry& e, std::string text) {
    keep_thread_result(text);  // (0.7, py-fixes) iad_last_result_json
    std::lock_guard<std::mutex> lock(e.mutex);
    e.last_result = std::move(text);
}

}  // namespace

iad_result IAD_CALL iad_cell_coord_plan_json(iad_cell* cell, const char* request, char* buffer, size_t buffer_size,
                                             size_t* required) {
    return guarded([&] {
        const auto e = find_cell(cell);
        check_text_output(buffer, buffer_size, required);
        const Request r = parse_request(request);
        coord::CoordinatedGroup group = e->cell->coordinated(r.axes);
        const coord::MotionPlan plan = group.plan(r.path, r.speed, r.accel_g, r.plan);
        write_text_output(with_summary(plan).dump(), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_cell_coord_run_json(iad_cell* cell, const char* request, char* buffer, size_t buffer_size,
                                            size_t* required) {
    return guarded([&] {
        const auto e = find_cell_to_run(cell);
        check_text_output(buffer, buffer_size, required);
        const Request r = parse_request(request);
        coord::CoordinatedGroup group = e->cell->coordinated(r.axes);
        std::string text;
        try {
            text = with_summary(group.run(r.path, r.speed, r.accel_g, r.plan, r.run)).dump();
        } catch (const coord::CoordinatedMotionError& error) {
            keep(*e, with_summary(error.result()).dump());  // the result so far (also in iad_last_error_json)
            throw;
        }
        keep(*e, text);  // kept even when the buffer is too small (a run is not repeated to read it)
        write_text_output(text, buffer, buffer_size, required);
    });
}
