// Private helpers shared by every implementation unit (header-only, internal: not installed, not part of the API).
// Formatting is locale-independent where it matters (hex, fixed decimals); numbers in messages use these helpers so
// the texts match the Python edition's ("0x0EE", "50.00", "0.1").
#pragma once

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

namespace iai_actuator_driver {
namespace util {

/// printf into a std::string.
inline std::string format(const char* fmt, ...)
#if defined(__GNUC__) || defined(__clang__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;

inline std::string format(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    va_list copy;
    va_copy(copy, args);
    const int size = std::vsnprintf(nullptr, 0, fmt, copy);
    va_end(copy);
    std::string out;
    if (size > 0) {
        out.resize(static_cast<std::size_t>(size) + 1);
        std::vsnprintf(&out[0], out.size(), fmt, args);
        out.resize(static_cast<std::size_t>(size));
    }
    va_end(args);
    return out;
}

/// "0x" + upper-case hex with at least `digits` digits: hex(0xEE, 3) = "0x0EE".
inline std::string hex(std::uint64_t value, int digits = 4) {
    return format("0x%0*llX", digits, static_cast<unsigned long long>(value));
}

/// Fixed decimals: fixed(50, 2) = "50.00".
inline std::string fixed(double value, int decimals = 2) { return format("%.*f", decimals, value); }

/// Shortest of %g style with up to 10 significant digits: g(0.1) = "0.1", g(50) = "50".
inline std::string g(double value) { return format("%.10g", value); }

/// Round to 0.01 (positions, speeds as the editions report them).
inline double round2(double value) { return std::round(value * 100.0) / 100.0; }

/// Python's repr() of a float: the shortest text that reads back as the same double, positional for exponents -4..15
/// with ".0" on whole numbers ("250.0", "0.01", "1e-05", "1e+16"), "inf" / "-inf" / "nan".
inline std::string py_float(double value) {
    if (std::isnan(value)) return "nan";
    if (std::isinf(value)) return value < 0 ? "-inf" : "inf";
    if (value == 0.0) return std::signbit(value) ? "-0.0" : "0.0";
    char buf[40];
    int precision = 1;
    for (; precision <= 17; ++precision) {
        std::snprintf(buf, sizeof buf, "%.*e", precision - 1, value);
        if (std::strtod(buf, nullptr) == value) break;
    }
    const std::string text = buf;  // "-d.ddde+XX"
    const bool negative = text[0] == '-';
    const std::size_t e = text.find('e');
    std::string digits;
    for (std::size_t i = negative ? 1 : 0; i < e; ++i)
        if (text[i] != '.') digits += text[i];
    while (digits.size() > 1 && digits.back() == '0') digits.pop_back();
    const int exponent = std::atoi(text.c_str() + e + 1);
    const int n = static_cast<int>(digits.size());
    std::string out = negative ? "-" : "";
    if (exponent >= -4 && exponent < 16) {
        if (exponent >= n - 1) {
            out += digits + std::string(static_cast<std::size_t>(exponent - (n - 1)), '0') + ".0";
        } else if (exponent >= 0) {
            out += digits.substr(0, static_cast<std::size_t>(exponent + 1)) + "." +
                   digits.substr(static_cast<std::size_t>(exponent + 1));
        } else {
            out += "0." + std::string(static_cast<std::size_t>(-exponent - 1), '0') + digits;
        }
        return out;
    }
    out += digits.substr(0, 1);
    if (n > 1) out += "." + digits.substr(1);
    return out + format("e%c%02d", exponent < 0 ? '-' : '+', exponent < 0 ? -exponent : exponent);
}

/// A number as Python prints it: an int (no decimal point) when `as_float` is false and the value is whole, else
/// py_float().
inline std::string py_number(double value, bool as_float) {
    if (!as_float && std::isfinite(value) && value == std::floor(value) && std::fabs(value) < 9.0e15)
        return format("%lld", static_cast<long long>(value));
    return py_float(value);
}

/// Python's round(value, decimals): the double nearest to the decimal value correctly rounded half to even (the
/// formatted decimal text read back).
inline double py_round(double value, int decimals) {
    if (!std::isfinite(value)) return value;
    char buf[400];
    std::snprintf(buf, sizeof buf, "%.*f", decimals, value);
    return std::strtod(buf, nullptr);
}

inline std::string join(const std::vector<std::string>& parts, const std::string& sep) {
    std::string out;
    for (std::size_t i = 0; i < parts.size(); ++i) {
        if (i) out += sep;
        out += parts[i];
    }
    return out;
}

inline std::string lower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

inline std::string upper(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return text;
}

inline bool iequals(const std::string& a, const std::string& b) { return lower(a) == lower(b); }

inline bool starts_with(const std::string& text, const std::string& prefix) {
    return text.size() >= prefix.size() && text.compare(0, prefix.size(), prefix) == 0;
}

inline std::string trim(const std::string& text) {
    std::size_t b = 0, e = text.size();
    while (b < e && std::isspace(static_cast<unsigned char>(text[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(text[e - 1]))) --e;
    return text.substr(b, e - b);
}

inline std::vector<std::string> split(const std::string& text, char sep) {
    std::vector<std::string> out;
    std::size_t start = 0;
    for (;;) {
        const std::size_t pos = text.find(sep, start);
        out.push_back(text.substr(start, pos == std::string::npos ? std::string::npos : pos - start));
        if (pos == std::string::npos) return out;
        start = pos + 1;
    }
}

/// The whole string as an integer ("0x" prefix = hex when base is 0), nullopt otherwise.
inline std::optional<long long> parse_int(const std::string& text, int base = 10) {
    const std::string t = trim(text);
    if (t.empty()) return std::nullopt;
    errno = 0;
    char* end = nullptr;
    const long long value = std::strtoll(t.c_str(), &end, base);
    if (errno != 0 || end == t.c_str() || *end != '\0') return std::nullopt;
    return value;
}

/// The whole string as a finite double ('.' decimal point), nullopt otherwise.
inline std::optional<double> parse_double(const std::string& text) {
    const std::string t = trim(text);
    if (t.empty()) return std::nullopt;
    errno = 0;
    char* end = nullptr;
    const double value = std::strtod(t.c_str(), &end);
    if (errno != 0 || end == t.c_str() || *end != '\0' || !std::isfinite(value)) return std::nullopt;
    return value;
}

/// Decode a hex string ("0A1B 2C3D" or "0A1B2C3D"; whitespace ignored) into 16-bit words (fixtures, tests).
inline std::vector<std::uint16_t> hex_words(const std::string& text) {
    std::string digits;
    for (char c : text)
        if (std::isxdigit(static_cast<unsigned char>(c))) digits += c;
    std::vector<std::uint16_t> words;
    for (std::size_t i = 0; i + 4 <= digits.size(); i += 4)
        words.push_back(static_cast<std::uint16_t>(std::strtoul(digits.substr(i, 4).c_str(), nullptr, 16)));
    return words;
}

/// Decode a hex string into bytes (whitespace ignored).
inline std::vector<std::uint8_t> hex_bytes(const std::string& text) {
    std::string digits;
    for (char c : text)
        if (std::isxdigit(static_cast<unsigned char>(c))) digits += c;
    std::vector<std::uint8_t> bytes;
    for (std::size_t i = 0; i + 2 <= digits.size(); i += 2)
        bytes.push_back(static_cast<std::uint8_t>(std::strtoul(digits.substr(i, 2).c_str(), nullptr, 16)));
    return bytes;
}

}  // namespace util
}  // namespace iai_actuator_driver
