// A minimal JSON value (header-only, no third-party dependency) and the JSON form of every public result type, for
// the CLI's --json output and for callers who log results. Object keys keep insertion order; copies are deep. The
// to_json overloads are implemented by the model unit (src/to_json.cpp); field names follow the Python edition's
// to_dict() so the outputs of the editions compare.
#pragma once

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace iai_actuator_driver {

class Json {
public:
    enum class Type { Null, Bool, Int, Number, String, Array, Object };

    Json() = default;                        ///< null
    Json(std::nullptr_t) {}                  ///< null
    Json(bool value) : type_(Type::Bool), bool_(value) {}
    Json(int value) : type_(Type::Int), int_(value) {}
    Json(long value) : type_(Type::Int), int_(value) {}
    Json(long long value) : type_(Type::Int), int_(value) {}
    Json(unsigned value) : type_(Type::Int), int_(static_cast<long long>(value)) {}
    Json(unsigned long value) : type_(Type::Int), int_(static_cast<long long>(value)) {}
    Json(unsigned long long value) : type_(Type::Int), int_(static_cast<long long>(value)) {}
    /// NaN / inf -> null.
    Json(double value) {
        if (std::isfinite(value)) {
            type_ = Type::Number;
            number_ = value;
        }
    }
    Json(const char* value) : type_(Type::String), string_(value ? value : "") {}
    Json(std::string value) : type_(Type::String), string_(std::move(value)) {}
    /// nullopt -> null, else the value.
    template <class T>
    Json(const std::optional<T>& value) : Json() {
        if (value) *this = Json(*value);
    }
    /// A vector -> an array of the converted elements.
    template <class T>
    Json(const std::vector<T>& values) : type_(Type::Array) {
        for (const auto& v : values) array_.push_back(Json(v));
    }

    /// A number as a reader that tells ints from floats (Python, C#) should get it: an integer when it is whole and
    /// `decimal` is false (a code, a count, a flag), else a float, written with a decimal point even when whole (a
    /// measure such as 150.0). NaN / inf -> null.
    static Json whole_or_number(double value, bool decimal) {
        if (!decimal && std::isfinite(value) && value == std::floor(value) && std::fabs(value) < 9.0e15)
            return Json(static_cast<long long>(value));
        return Json(value);
    }

    static Json array() {
        Json j;
        j.type_ = Type::Array;
        return j;
    }
    static Json object() {
        Json j;
        j.type_ = Type::Object;
        return j;
    }

    Type type() const { return type_; }
    bool is_null() const { return type_ == Type::Null; }

    /// Array: append (a null becomes an array). std::logic_error on another type.
    Json& push_back(Json value) {
        if (type_ == Type::Null) type_ = Type::Array;
        if (type_ != Type::Array) throw std::logic_error("Json::push_back on a non-array");
        array_.push_back(std::move(value));
        return *this;
    }
    /// Object: set, replacing an existing key in place (a null becomes an object). std::logic_error on another type.
    Json& set(const std::string& key, Json value) {
        if (type_ == Type::Null) type_ = Type::Object;
        if (type_ != Type::Object) throw std::logic_error("Json::set on a non-object");
        for (auto& kv : object_)
            if (kv.first == key) {
                kv.second = std::move(value);
                return *this;
            }
        object_.emplace_back(key, std::move(value));
        return *this;
    }
    /// Object lookup, nullptr when absent or not an object.
    const Json* find(const std::string& key) const {
        if (type_ != Type::Object) return nullptr;
        for (const auto& kv : object_)
            if (kv.first == key) return &kv.second;
        return nullptr;
    }
    /// Array / object size, 0 otherwise.
    std::size_t size() const {
        return type_ == Type::Array ? array_.size() : type_ == Type::Object ? object_.size() : 0;
    }

    bool as_bool() const { return require(Type::Bool), bool_; }
    /// An Int, or a Number truncated toward zero (held at the long long range: converting a double outside it is
    /// undefined). A caller that must refuse a fraction or an out-of-range value uses as_whole().
    long long as_int() const {
        if (type_ == Type::Number) {
            if (number_ >= 9223372036854775808.0) return 9223372036854775807LL;
            if (number_ < -9223372036854775808.0) return -9223372036854775807LL - 1;
            return static_cast<long long>(number_);
        }
        return require(Type::Int), int_;
    }
    /// The value as a whole number: an Int, or a Number that is whole and inside the long long range (9600.0 ->
    /// 9600); nullopt for a fraction, a number past the range (1e300) and every other type. Range-check the result
    /// before narrowing it to a smaller type.
    std::optional<long long> as_whole() const {
        if (type_ == Type::Int) return int_;
        if (type_ == Type::Number && number_ == std::floor(number_) && number_ >= -9223372036854775808.0 &&
            number_ < 9223372036854775808.0)
            return static_cast<long long>(number_);
        return std::nullopt;
    }
    double as_number() const {
        if (type_ == Type::Int) return static_cast<double>(int_);
        return require(Type::Number), number_;
    }
    const std::string& as_string() const { return require(Type::String), string_; }
    const std::vector<Json>& as_array() const { return require(Type::Array), array_; }
    const std::vector<std::pair<std::string, Json>>& as_object() const { return require(Type::Object), object_; }

    /// Compact text (indent < 0) or pretty-printed with `indent` spaces. Integers as integers; other numbers with up
    /// to 10 significant digits, and with a decimal point when they happen to be whole (1.0, not 1), so a reader in
    /// another language keeps them a float, as the Python edition's json.dumps writes them.
    std::string dump(int indent = -1) const {
        std::string out;
        write(out, indent, 0);
        return out;
    }

    /// Parse JSON text (std::invalid_argument on a syntax error).
    static Json parse(const std::string& text) {
        std::size_t pos = 0;
        Json value = parse_value(text, pos);
        skip_ws(text, pos);
        if (pos != text.size()) throw std::invalid_argument("Json::parse: trailing characters");
        return value;
    }

    bool operator==(const Json& o) const {
        if (type_ != o.type_) {
            if ((type_ == Type::Int || type_ == Type::Number) && (o.type_ == Type::Int || o.type_ == Type::Number))
                return as_number() == o.as_number();
            return false;
        }
        switch (type_) {
            case Type::Null: return true;
            case Type::Bool: return bool_ == o.bool_;
            case Type::Int: return int_ == o.int_;
            case Type::Number: return number_ == o.number_;
            case Type::String: return string_ == o.string_;
            case Type::Array: return array_ == o.array_;
            default: return object_ == o.object_;
        }
    }
    bool operator!=(const Json& o) const { return !(*this == o); }

private:
    void require(Type t) const {
        if (type_ != t) throw std::logic_error("Json: wrong type");
    }

    static void quote(std::string& out, const std::string& s) {
        out += '"';
        for (unsigned char c : s) {
            switch (c) {
                case '"': out += "\\\""; break;
                case '\\': out += "\\\\"; break;
                case '\n': out += "\\n"; break;
                case '\r': out += "\\r"; break;
                case '\t': out += "\\t"; break;
                default:
                    if (c < 0x20) {
                        char buf[8];
                        std::snprintf(buf, sizeof buf, "\\u%04x", c);
                        out += buf;
                    } else {
                        out += static_cast<char>(c);
                    }
            }
        }
        out += '"';
    }

    static void newline(std::string& out, int indent, int depth) {
        if (indent < 0) return;
        out += '\n';
        out.append(static_cast<std::size_t>(indent * depth), ' ');
    }

    void write(std::string& out, int indent, int depth) const {
        switch (type_) {
            case Type::Null: out += "null"; break;
            case Type::Bool: out += bool_ ? "true" : "false"; break;
            case Type::Int: out += std::to_string(int_); break;
            case Type::Number: {
                char buf[32];
                std::snprintf(buf, sizeof buf, "%.10g", number_);
                out += buf;
                bool plain = true;  // no '.' and no exponent: a whole number, which must still read as a float
                for (const char* c = buf; *c; ++c)
                    if (*c == '.' || *c == ',' || *c == 'e' || *c == 'E') plain = false;
                if (plain) out += ".0";
                break;
            }
            case Type::String: quote(out, string_); break;
            case Type::Array: {
                if (array_.empty()) {
                    out += "[]";
                    break;
                }
                out += '[';
                for (std::size_t i = 0; i < array_.size(); ++i) {
                    if (i) out += ',';
                    newline(out, indent, depth + 1);
                    array_[i].write(out, indent, depth + 1);
                }
                newline(out, indent, depth);
                out += ']';
                break;
            }
            case Type::Object: {
                if (object_.empty()) {
                    out += "{}";
                    break;
                }
                out += '{';
                for (std::size_t i = 0; i < object_.size(); ++i) {
                    if (i) out += ',';
                    newline(out, indent, depth + 1);
                    quote(out, object_[i].first);
                    out += indent < 0 ? ":" : ": ";
                    object_[i].second.write(out, indent, depth + 1);
                }
                newline(out, indent, depth);
                out += '}';
                break;
            }
        }
    }

    static void skip_ws(const std::string& t, std::size_t& p) {
        while (p < t.size() && (t[p] == ' ' || t[p] == '\n' || t[p] == '\r' || t[p] == '\t')) ++p;
    }
    static void expect(const std::string& t, std::size_t& p, const char* word) {
        for (std::size_t i = 0; word[i]; ++i, ++p)
            if (p >= t.size() || t[p] != word[i]) throw std::invalid_argument("Json::parse: unexpected text");
    }
    static std::string parse_string(const std::string& t, std::size_t& p) {
        if (p >= t.size() || t[p] != '"') throw std::invalid_argument("Json::parse: string expected");
        ++p;
        std::string s;
        while (p < t.size() && t[p] != '"') {
            char c = t[p++];
            if (c != '\\') {
                s += c;
                continue;
            }
            if (p >= t.size()) break;
            const char e = t[p++];
            switch (e) {
                case 'n': s += '\n'; break;
                case 'r': s += '\r'; break;
                case 't': s += '\t'; break;
                case 'b': s += '\b'; break;
                case 'f': s += '\f'; break;
                case 'u': {
                    if (p + 4 > t.size()) throw std::invalid_argument("Json::parse: bad \\u escape");
                    const unsigned code = static_cast<unsigned>(std::strtoul(t.substr(p, 4).c_str(), nullptr, 16));
                    p += 4;
                    if (code < 0x80) {
                        s += static_cast<char>(code);
                    } else if (code < 0x800) {
                        s += static_cast<char>(0xC0 | (code >> 6));
                        s += static_cast<char>(0x80 | (code & 0x3F));
                    } else {
                        s += static_cast<char>(0xE0 | (code >> 12));
                        s += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
                        s += static_cast<char>(0x80 | (code & 0x3F));
                    }
                    break;
                }
                default: s += e;
            }
        }
        if (p >= t.size()) throw std::invalid_argument("Json::parse: unterminated string");
        ++p;
        return s;
    }
    static Json parse_value(const std::string& t, std::size_t& p) {
        skip_ws(t, p);
        if (p >= t.size()) throw std::invalid_argument("Json::parse: value expected");
        const char c = t[p];
        if (c == 'n') return expect(t, p, "null"), Json();
        if (c == 't') return expect(t, p, "true"), Json(true);
        if (c == 'f') return expect(t, p, "false"), Json(false);
        if (c == '"') return Json(parse_string(t, p));
        if (c == '[') {
            ++p;
            Json a = array();
            skip_ws(t, p);
            if (p < t.size() && t[p] == ']') return ++p, a;
            for (;;) {
                a.push_back(parse_value(t, p));
                skip_ws(t, p);
                if (p < t.size() && t[p] == ',') {
                    ++p;
                    continue;
                }
                if (p < t.size() && t[p] == ']') return ++p, a;
                throw std::invalid_argument("Json::parse: ',' or ']' expected");
            }
        }
        if (c == '{') {
            ++p;
            Json o = object();
            skip_ws(t, p);
            if (p < t.size() && t[p] == '}') return ++p, o;
            for (;;) {
                skip_ws(t, p);
                std::string key = parse_string(t, p);
                skip_ws(t, p);
                if (p >= t.size() || t[p] != ':') throw std::invalid_argument("Json::parse: ':' expected");
                ++p;
                o.set(key, parse_value(t, p));
                skip_ws(t, p);
                if (p < t.size() && t[p] == ',') {
                    ++p;
                    continue;
                }
                if (p < t.size() && t[p] == '}') return ++p, o;
                throw std::invalid_argument("Json::parse: ',' or '}' expected");
            }
        }
        // number
        const std::size_t start = p;
        if (p < t.size() && (t[p] == '-' || t[p] == '+')) ++p;
        bool is_float = false;
        while (p < t.size() && ((t[p] >= '0' && t[p] <= '9') || t[p] == '.' || t[p] == 'e' || t[p] == 'E' ||
                                ((t[p] == '-' || t[p] == '+') && (t[p - 1] == 'e' || t[p - 1] == 'E')))) {
            if (t[p] == '.' || t[p] == 'e' || t[p] == 'E') is_float = true;
            ++p;
        }
        if (p == start) throw std::invalid_argument("Json::parse: unexpected character");
        const std::string num = t.substr(start, p - start);
        // a number the value type cannot hold is refused, never narrowed (LLONG_MAX) or turned into null (inf)
        errno = 0;
        if (!is_float) {
            const long long whole = std::strtoll(num.c_str(), nullptr, 10);
            if (errno == ERANGE) throw std::invalid_argument("Json::parse: integer out of range: " + num);
            return Json(whole);
        }
        const double number = std::strtod(num.c_str(), nullptr);
        if (!std::isfinite(number)) throw std::invalid_argument("Json::parse: number out of range: " + num);
        return Json(number);
    }

    Type type_ = Type::Null;
    bool bool_ = false;
    long long int_ = 0;
    double number_ = 0.0;
    std::string string_;
    std::vector<Json> array_;
    std::vector<std::pair<std::string, Json>> object_;
};

// ------------------------------------------------------------------------------------------ to_json (model unit)
struct Status;
struct AlarmRecord;
struct AlarmInfo;
struct DeviceInfo;
struct AxisLimits;
struct LimitRange;
struct AxisRanges;
struct PositionEntry;
struct ParamValue;
struct ParameterInfo;
struct ParameterModel;
struct ParameterWriteResult;
struct PositionWriteResult;
struct SoftwareResetResult;
struct BaudChangeResult;
struct PortBaudResult;
struct ScanHit;
struct SerialPortInfo;
struct IndexCategory;
struct Span;

Json to_json(const Status& value);            ///< fields + derived flags + raw words by name
Json to_json(const AlarmRecord& value);
Json to_json(const AlarmInfo& value);
Json to_json(const DeviceInfo& value);
Json to_json(const AxisLimits& value);
Json to_json(const LimitRange& value);
Json to_json(const AxisRanges& value);
Json to_json(const PositionEntry& value);
Json to_json(const ParamValue& value);        ///< the number, or the text
Json to_json(const ParameterInfo& value);
Json to_json(const ParameterModel& value);
Json to_json(const ParameterWriteResult& value);  ///< + saved, needs_reset, takes_effect
Json to_json(const PositionWriteResult& value);
Json to_json(const SoftwareResetResult& value);
Json to_json(const BaudChangeResult& value);
Json to_json(const PortBaudResult& value);
Json to_json(const ScanHit& value);           ///< + name, axis_number, product_family
Json to_json(const SerialPortInfo& value);    ///< + is_iai
Json to_json(const IndexCategory& value);
Json to_json(const Span& value);              ///< [min, max]

}  // namespace iai_actuator_driver
