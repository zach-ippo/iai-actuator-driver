// A tiny self-contained test harness (no third-party dependency).
//
//   #include "harness.hpp"
//   TEST(bus_crc_known_vector) {
//       CHECK_EQ(iai_actuator_driver::crc16(data), 0x4D12);
//       CHECK(x > 0);
//       CHECK_NEAR(st.position_mm, 50.0, 0.01);
//       CHECK_THROWS(ax.move_to(1e9), iai_actuator_driver::SoftLimitError);   // that type (or a subclass)
//       CHECK_THROWS(f());                                          // any exception
//       REQUIRE(ptr != nullptr);                                     // stops this test when false
//   }
//
// Test names are identifiers and start with their unit: bus_, model_, sim_, axis_, cell_, cli_ (the runner filters
// on name prefixes). A failed CHECK records the failure and the test goes on; an exception escaping a test fails it.
// A test that needs another thread uses ec_test::Thread, not a raw std::thread (an exception escaping it fails the
// test instead of ending the process).
// main() is in test_main.cpp: `ec_tests [-l] [--quiet] [--shard K/N] [PREFIX...]` runs every test whose name starts
// with one of the prefixes (all when none) and prints a summary; the exit code is 0 only when every test passed.
// --shard K/N keeps every N-th of those tests, starting at the K-th (1 <= K <= N): N processes with K = 1..N run each
// selected test exactly once (core/tests/run_shards.py starts them and merges their output). -l lists the selection.
#pragma once

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace ec_test {

struct TestCase {
    const char* name;
    void (*fn)();
    const char* file;
    int line;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

struct Registrar {
    Registrar(const char* name, void (*fn)(), const char* file, int line) { registry().push_back({name, fn, file, line}); }
};

/// State of the running test.
struct Current {
    const char* name = nullptr;
    int failures = 0;
    long checks = 0;
};

inline Current& current() {
    static Current c;
    return c;
}

/// Called before / after each test with its name, or null (test_main.cpp sets them: EC_TESTS_THREADS=1 reports the
/// threads a test leaves running).
using TestHook = void (*)(const char* name);
inline TestHook& before_each() {
    static TestHook hook = nullptr;
    return hook;
}
inline TestHook& after_each() {
    static TestHook hook = nullptr;
    return hook;
}

/// Thrown by REQUIRE to end the test.
struct RequireFailed {};

inline void fail(const char* file, int line, const std::string& message) {
    ++current().failures;
    std::cerr << "  FAIL " << (current().name ? current().name : "?") << " (" << file << ":" << line << "): " << message
              << "\n";
}

// ---- value printing for CHECK_EQ: streams what can be streamed, otherwise "<value>"
template <class T, class = void>
struct is_streamable : std::false_type {};
template <class T>
struct is_streamable<T, std::void_t<decltype(std::declval<std::ostream&>() << std::declval<const T&>())>>
    : std::true_type {};

template <class T>
std::string show(const T& value) {
    if constexpr (std::is_same<T, bool>::value) {
        return value ? "true" : "false";
    } else if constexpr (std::is_same<T, unsigned char>::value || std::is_same<T, signed char>::value ||
                         std::is_same<T, char>::value) {
        return std::to_string(static_cast<int>(value));
    } else if constexpr (std::is_same<T, std::nullptr_t>::value) {
        return "nullptr";
    } else if constexpr (is_streamable<T>::value) {
        std::ostringstream s;
        s.precision(10);
        s << value;
        return s.str();
    } else {
        return "<value>";
    }
}

template <class A, class B>
bool check_eq(const A& a, const B& b, const char* ea, const char* eb, const char* file, int line) {
    ++current().checks;
    bool equal;
    if constexpr (std::is_arithmetic<A>::value && std::is_arithmetic<B>::value) {
        if constexpr (std::is_integral<A>::value && std::is_integral<B>::value) {
            // compare integers by value regardless of signedness
            equal = (static_cast<long double>(a) == static_cast<long double>(b));
        } else {
            equal = (a == b);
        }
    } else {
        equal = (a == b);
    }
    if (!equal) fail(file, line, std::string("CHECK_EQ(") + ea + ", " + eb + "): " + show(a) + " != " + show(b));
    return equal;
}

inline bool check_near(double a, double b, double tol, const char* ea, const char* eb, const char* file, int line) {
    ++current().checks;
    const bool ok = std::fabs(a - b) <= tol;
    if (!ok)
        fail(file, line, std::string("CHECK_NEAR(") + ea + ", " + eb + "): " + show(a) + " vs " + show(b) +
                             " (tolerance " + show(tol) + ")");
    return ok;
}

// The caller's file and line as default arguments, where the compiler offers them.
#if defined(__clang__) || defined(__GNUC__) || (defined(_MSC_VER) && _MSC_VER >= 1926)
#define EC_TEST_CALLER_FILE __builtin_FILE()
#define EC_TEST_CALLER_LINE __builtin_LINE()
#else
#define EC_TEST_CALLER_FILE "?"
#define EC_TEST_CALLER_LINE 0
#endif

/// What an exception says: its what() text ("" for RequireFailed, which REQUIRE has reported already).
inline std::string describe(const std::exception_ptr& error) {
    try {
        std::rethrow_exception(error);
    } catch (const RequireFailed&) {
        return "";
    } catch (const std::exception& e) {
        return e.what();
    } catch (...) {
        return "non-standard exception";
    }
}

/// A test's own thread, instead of a raw std::thread. Runs `fn` under catch(...) and joins in its destructor, so
/// neither an exception from the body (std::terminate in a raw std::thread) nor a test that ends early (a failed
/// REQUIRE, an unexpected exception: a joinable std::thread destroyed) ends the process. What the body threw is kept:
/// join() or the destructor reports it as a failure of the current test with its what() text, unless the test took it
/// with error() to look at it as data. Not for a body that only ends when told to (the destructor would wait for it).
///
///   ec_test::Thread worker([&] { result = group.run(path, 20, 0.3); });
///   ...
///   worker.join();                               // a failure of this test when run() threw
///   const std::exception_ptr e = worker.error(); // or: joined, and what run() threw (null: nothing) is the test's
class Thread {
public:
    template <class F>
    explicit Thread(F&& fn, const char* file = EC_TEST_CALLER_FILE, int line = EC_TEST_CALLER_LINE)
        : file_(file), line_(line), thread_([this, body = std::forward<F>(fn)]() mutable {
              try {
                  body();
              } catch (...) {
                  error_ = std::current_exception();
              }
          }) {}
    Thread(const Thread&) = delete;
    Thread& operator=(const Thread&) = delete;
    ~Thread() {
        try {
            join();
        } catch (...) {
        }
    }

    /// Wait for the body; when it threw, a failure of the current test (reported once).
    void join() {
        wait();
        if (!error_) return;
        const std::string what = describe(std::exchange(error_, nullptr));
        if (!what.empty()) fail(file_, line_, "exception in a test thread: " + what);
    }

    /// Wait for the body and take what it threw (null when it returned); nothing is reported, the test looks at it.
    std::exception_ptr error() {
        wait();
        return std::exchange(error_, nullptr);
    }

private:
    void wait() {
        if (thread_.joinable()) thread_.join();
    }
    const char* file_;
    int line_;
    std::exception_ptr error_;  // before thread_: set by the body, read only after the join
    std::thread thread_;
};

/// A slice of the selection: every `count`-th selected test, starting at the `index`-th (0-based). {0, 1}: all.
struct Shard {
    int index = 0;
    int count = 1;
};

/// The registered tests whose name starts with one of `prefixes` (all when empty; `exact`: whose name is one of them),
/// less those named in `excluded`, in registration order, then the shard's slice of them (round robin, so the slow
/// tests of one file spread over the shards).
inline std::vector<const TestCase*> select_tests(const std::vector<std::string>& prefixes, Shard shard = {},
                                                 const std::vector<std::string>& excluded = {}, bool exact = false) {
    std::vector<const TestCase*> out;
    int position = 0;
    for (const TestCase& t : registry()) {
        bool selected = prefixes.empty();
        for (const auto& p : prefixes)
            if (exact ? p == t.name : std::strncmp(t.name, p.c_str(), p.size()) == 0) selected = true;
        for (const auto& name : excluded)
            if (name == t.name) selected = false;
        if (!selected) continue;
        if (position++ % shard.count == shard.index) out.push_back(&t);
    }
    return out;
}

/// Run `tests` and print a summary. Returns the exit code: 0 when every test passed (and, unless `empty_ok`, at
/// least one ran).
inline int run_tests(const std::vector<const TestCase*>& tests, bool quiet = false, bool empty_ok = false) {
    int passed = 0, failed = 0, run = 0;
    long checks = 0;
    std::vector<std::string> failed_names;
    const auto t0 = std::chrono::steady_clock::now();
    for (const TestCase* tc : tests) {
        const TestCase& t = *tc;
        ++run;
        current() = Current{t.name, 0, 0};
        if (before_each()) before_each()(t.name);
        const auto start = std::chrono::steady_clock::now();
        try {
            t.fn();
        } catch (const RequireFailed&) {
            // already reported
        } catch (const std::exception& e) {
            fail(t.file, t.line, std::string("unexpected exception: ") + e.what());
        } catch (...) {
            fail(t.file, t.line, "unexpected non-standard exception");
        }
        const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        if (after_each()) after_each()(t.name);
        checks += current().checks;
        if (current().failures == 0) {
            ++passed;
            if (!quiet) std::printf("  ok   %-60s %6.3f s\n", t.name, secs);
        } else {
            ++failed;
            failed_names.push_back(t.name);
            std::printf("  FAIL %-60s %6.3f s\n", t.name, secs);
        }
        std::fflush(stdout);
    }
    const double total = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("\n%d tests, %d passed, %d failed, %ld checks, %.2f s\n", run, passed, failed, checks, total);
    for (const auto& n : failed_names) std::printf("  failed: %s\n", n.c_str());
    if (run == 0) std::printf(empty_ok ? "no test in this shard\n" : "no test matched\n");
    return failed == 0 && (run > 0 || empty_ok) ? 0 : 1;
}

/// Run the registered tests whose name starts with one of `prefixes` (all when empty). Returns the exit code.
inline int run_all(const std::vector<std::string>& prefixes, bool quiet = false) {
    return run_tests(select_tests(prefixes), quiet);
}

/// Parse "K/N" (1 <= K <= N) into a Shard; false when it is not that.
inline bool parse_shard(const std::string& text, Shard& shard) {
    const auto slash = text.find('/');
    if (slash == std::string::npos || slash == 0 || slash + 1 >= text.size()) return false;
    const std::string k = text.substr(0, slash), n = text.substr(slash + 1);
    if (k.find_first_not_of("0123456789") != std::string::npos || n.find_first_not_of("0123456789") != std::string::npos ||
        k.size() > 6 || n.size() > 6)
        return false;
    const int ki = std::stoi(k), ni = std::stoi(n);
    if (ni < 1 || ki < 1 || ki > ni) return false;
    shard = Shard{ki - 1, ni};
    return true;
}

/// main() of the test executable (test_main.cpp): [-l | --list] [--quiet] [--shard K/N] [--exclude NAME]... [--exact]
/// [PREFIX...]. --exclude leaves out the test of that name (run_shards.py runs it in its serial tail); --exact takes
/// each PREFIX as a whole test name.
inline int main_entry(int argc, char** argv) {
    std::vector<std::string> prefixes, excluded;
    bool quiet = false, list = false, sharded = false, exact = false;
    Shard shard;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-l" || arg == "--list") {
            list = true;
        } else if (arg == "--quiet" || arg == "-q") {
            quiet = true;
        } else if (arg == "--exact") {
            exact = true;
        } else if (arg == "--exclude" || arg.rfind("--exclude=", 0) == 0) {
            const std::string value = arg == "--exclude" ? (i + 1 < argc ? argv[++i] : "") : arg.substr(10);
            if (value.empty()) {
                std::fprintf(stderr, "--exclude takes a test name\n");
                return 2;
            }
            excluded.push_back(value);
        } else if (arg == "--shard" || arg.rfind("--shard=", 0) == 0) {
            const std::string value = arg == "--shard" ? (i + 1 < argc ? argv[++i] : "") : arg.substr(8);
            if (!parse_shard(value, shard)) {
                std::fprintf(stderr, "--shard takes K/N with 1 <= K <= N, not '%s'\n", value.c_str());
                return 2;
            }
            sharded = true;
        } else if (!arg.empty() && arg[0] == '-') {
            std::fprintf(stderr,
                         "unknown option %s (usage: ec_tests [-l] [--quiet] [--shard K/N] [--exclude NAME]... "
                         "[--exact] [PREFIX...])\n",
                         arg.c_str());
            return 2;
        } else {
            prefixes.push_back(arg);
        }
    }
    const std::vector<const TestCase*> tests = select_tests(prefixes, shard, excluded, exact);
    if (list) {
        for (const TestCase* t : tests) std::printf("%s\n", t->name);
        return 0;
    }
    // a shard may be empty (fewer tests than shards): the runner checks the shards together ran the whole selection
    return run_tests(tests, quiet, sharded);
}

}  // namespace ec_test

#define EC_TEST_CAT2(a, b) a##b
#define EC_TEST_CAT(a, b) EC_TEST_CAT2(a, b)

/// Define and register a test. `name` is an identifier starting with its unit (bus_, model_, sim_, axis_, cell_, cli_).
#define TEST(name)                                                                               \
    static void EC_TEST_CAT(ec_test_fn_, name)();                                                \
    static ::ec_test::Registrar EC_TEST_CAT(ec_test_reg_, name)(#name, &EC_TEST_CAT(ec_test_fn_, name), \
                                                               __FILE__, __LINE__);               \
    static void EC_TEST_CAT(ec_test_fn_, name)()

/// Record a failure when `cond` is false; the test goes on.
#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        ++::ec_test::current().checks;                                                                       \
        if (!(cond)) ::ec_test::fail(__FILE__, __LINE__, "CHECK(" #cond ")");                                \
    } while (0)

/// End the test when `cond` is false.
#define REQUIRE(cond)                                                                                        \
    do {                                                                                                     \
        ++::ec_test::current().checks;                                                                       \
        if (!(cond)) {                                                                                       \
            ::ec_test::fail(__FILE__, __LINE__, "REQUIRE(" #cond ")");                                       \
            throw ::ec_test::RequireFailed{};                                                                \
        }                                                                                                    \
    } while (0)

/// a == b (integers compare by value across signedness); prints both values on failure.
#define CHECK_EQ(a, b) ((void)::ec_test::check_eq((a), (b), #a, #b, __FILE__, __LINE__))

/// |a - b| <= tol.
#define CHECK_NEAR(a, b, tol) ((void)::ec_test::check_near((a), (b), (tol), #a, #b, __FILE__, __LINE__))

#define EC_TEST_THROWS_ANY(expr)                                                                             \
    do {                                                                                                     \
        ++::ec_test::current().checks;                                                                       \
        bool ec_test_threw = false;                                                                          \
        try {                                                                                                \
            (void)(expr);                                                                                    \
        } catch (...) {                                                                                      \
            ec_test_threw = true;                                                                            \
        }                                                                                                    \
        if (!ec_test_threw) ::ec_test::fail(__FILE__, __LINE__, "CHECK_THROWS(" #expr "): nothing thrown");  \
    } while (0)

#define EC_TEST_THROWS_AS(expr, type)                                                                        \
    do {                                                                                                     \
        ++::ec_test::current().checks;                                                                       \
        bool ec_test_ok = false;                                                                             \
        std::string ec_test_other;                                                                           \
        try {                                                                                                \
            (void)(expr);                                                                                    \
            ec_test_other = "nothing thrown";                                                                \
        } catch (const type&) {                                                                              \
            ec_test_ok = true;                                                                               \
        } catch (const std::exception& ec_test_e) {                                                          \
            ec_test_other = std::string("other exception: ") + ec_test_e.what();                             \
        } catch (...) {                                                                                      \
            ec_test_other = "other non-standard exception";                                                  \
        }                                                                                                    \
        if (!ec_test_ok)                                                                                     \
            ::ec_test::fail(__FILE__, __LINE__, "CHECK_THROWS(" #expr ", " #type "): " + ec_test_other);     \
    } while (0)

#define EC_TEST_PICK(_1, _2, NAME, ...) NAME
/// CHECK_THROWS(expr) - any exception; CHECK_THROWS(expr, Type) - Type or a subclass.
#define CHECK_THROWS(...) EC_TEST_PICK(__VA_ARGS__, EC_TEST_THROWS_AS, EC_TEST_THROWS_ANY, unused)(__VA_ARGS__)
