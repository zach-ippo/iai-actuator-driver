// Command-line parsing of iai-actuator-driver-cpp (port of the C# CliArgs.cs; the options of the Python cli.py).
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "cli.hpp"
#include "util.hpp"

namespace iai_actuator_driver {
namespace cli {

namespace {

bool is_number_token(const std::string& token) {
    if (token.size() < 2 || token[0] != '-') return false;
    return util::parse_double(token).has_value();
}

}  // namespace

const std::map<std::string, int>& Args::arity() {
    static const std::map<std::string, int> table{
        // flags
        {"--help", 0},
        {"--version", 0},
        {"--json", 0},
        {"--sim", 0},
        {"--log-frames", 0},
        {"--no-wait", 0},
        {"--force", 0},
        {"--no-commit", 0},
        {"--allow-fast", 0},
        // hidden: accepted and ignored (no longer needed), not in --help
        {"--allow-nonvolatile", 0},
        {"--allow-dangerous", 0},
        {"--allow-home-params", 0},
        {"--allow-unlisted", 0},
        {"--allow-unverified-code", 0},
        {"--unit-max", 0},
        {"--fast", 0},
        {"--toward-home", 0},
        {"--watch", 0},
        {"--monitor", 0},
        {"--motion", 0},
        {"--no-control", 0},
        {"--broadcast", 0},
        // a flag with an optional on / off value (kOnOff)
        {"--safety", kOnOff},
        // one value
        {"--decel", 1},
        {"--accel-pattern", 1},
        {"--change-speed", 1},
        {"--change-accel", 1},
        {"--change-after", 1},
        {"--count", 1},
        {"--interval", 1},
        {"--duration", 1},
        {"--section", 1},
        {"--rows", 1},
        {"--backup", 1},
        {"--target", 1},
        {"--restore-baud", 1},
        {"--wait", 1},
        {"--report", 1},
        {"--line", 1},
        {"--position-unit", 1},
        {"--sim-line", 1},
        {"--sim-unit", 1},
        {"--port", 1},
        {"--ports", 1},
        {"--sim-profile", 1},
        {"--slave", 1},
        {"--axis", 1},
        {"--baud", 1},
        {"--timeout", 1},
        {"--retries", 1},
        {"--speed", 1},
        {"--accel", 1},
        {"--band", 1},
        {"--method", 1},
        {"--strategy", 1},
        {"--speed-pct", 1},
        {"--slaves", 1},
        {"--scan-timeout", 1},
        {"--max-speed", 1},
        {"--max-accel", 1},
        {"--max-push", 1},
        {"--sim-time-scale", 1},
        {"--sim-baud", 1},
        {"--sim-state", 1},
        {"--sim-slaves", 1},
        // two values
        {"--stroke", 2},
    };
    return table;
}

Args Args::parse(const std::vector<std::string>& argv) {
    Args args;
    const auto& table = arity();
    for (std::size_t i = 0; i < argv.size(); ++i) {
        std::string token = argv[i];
        if (token == "-h") token = "--help";
        if (token.size() < 2 || token[0] != '-' || is_number_token(token) || token == "--") {
            if (token != "--") args.positionals_.push_back(token);
            continue;
        }
        std::string name = token;
        std::optional<std::string> inline_value;
        const std::size_t eq = token.find('=');
        if (eq != std::string::npos) {
            name = token.substr(0, eq);
            inline_value = token.substr(eq + 1);
        }
        const auto it = table.find(name);
        if (it == table.end()) throw UsageError("unknown option " + name);
        std::vector<std::string> values;
        if (inline_value) {
            if (it->second == kOnOff) {
                if (*inline_value != "on" && *inline_value != "off")
                    throw UsageError(name + ": on or off, got '" + *inline_value + "'");
            } else if (it->second != 1) {
                throw UsageError(name + " takes " + std::to_string(it->second) + " value(s)");
            }
            values.push_back(*inline_value);
        } else if (it->second == kOnOff) {
            // "--safety", "--safety on", "--safety off": the next token is taken only when it is on / off
            if (i + 1 < argv.size() && (argv[i + 1] == "on" || argv[i + 1] == "off")) values.push_back(argv[++i]);
        } else {
            for (int k = 0; k < it->second; ++k) {
                if (i + 1 >= argv.size()) throw UsageError(name + " needs a value");
                const std::string& next = argv[i + 1];
                if (next.size() > 1 && next[0] == '-' && !is_number_token(next) && next != "-")
                    throw UsageError(name + " needs a value");
                values.push_back(next);
                ++i;
            }
        }
        auto& all = args.every_[name];
        all.insert(all.end(), values.begin(), values.end());
        args.options_[name] = values;
    }
    return args;
}

bool Args::has(const std::string& name) const { return options_.count(name) != 0; }

std::optional<std::string> Args::value(const std::string& name) const {
    const auto it = options_.find(name);
    if (it == options_.end() || it->second.empty()) return std::nullopt;
    return it->second.front();
}

std::vector<std::string> Args::values(const std::string& name) const {
    const auto it = options_.find(name);
    return it == options_.end() ? std::vector<std::string>() : it->second;
}

std::vector<std::string> Args::every(const std::string& name) const {
    const auto it = every_.find(name);
    return it == every_.end() ? std::vector<std::string>() : it->second;
}

std::optional<double> Args::number(const std::string& name) const {
    const auto v = value(name);
    if (!v) return std::nullopt;
    return parse_number(name, *v);
}

std::optional<int> Args::integer(const std::string& name) const {
    const auto v = value(name);
    if (!v) return std::nullopt;
    return parse_integer(name, *v);
}

void Args::allow(const std::string& command, const std::vector<std::string>& allowed) const {
    for (const auto& kv : options_) {
        if (kv.first == "--help" || kv.first == "--version" || kv.first == "--json") continue;
        if (std::find(allowed.begin(), allowed.end(), kv.first) == allowed.end())
            throw UsageError("'" + command + "' does not take " + kv.first);
    }
}

double parse_number(const std::string& what, const std::string& text) {
    const auto v = util::parse_double(text);
    if (!v) throw UsageError(what + ": not a number: '" + text + "'");
    return *v;
}

int parse_integer(const std::string& what, const std::string& text) {
    const auto v = util::parse_int(text);
    if (!v || *v < -2147483647LL || *v > 2147483647LL) throw UsageError(what + ": not an integer: '" + text + "'");
    return static_cast<int>(*v);
}

std::vector<int> parse_slave_list(const std::string& what, const std::string& text) {
    std::vector<int> out;
    for (const std::string& raw : util::split(text, ',')) {
        const std::string part = util::trim(raw);
        if (part.empty()) continue;
        const std::size_t dash = part.find('-', 1);
        const int first = parse_integer(what, dash == std::string::npos ? part : part.substr(0, dash));
        const int last = dash == std::string::npos ? first : parse_integer(what, part.substr(dash + 1));
        if (!(1 <= first && first <= last && last <= 247))
            throw UsageError(what + ": bad slave range '" + part + "' (addresses 1..247)");
        for (int s = first; s <= last; ++s)
            if (std::find(out.begin(), out.end(), s) == out.end()) out.push_back(s);
    }
    if (out.empty()) throw UsageError(what + ": no slave address given");
    return out;
}

}  // namespace cli
}  // namespace iai_actuator_driver
