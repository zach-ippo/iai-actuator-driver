// Entry point of ec_tests: runs every TEST registered by the linked test files (see harness.hpp).
//
// Diagnostics, set by the environment:
//   EC_TESTS_POISON     on unless set to 0 (and off by default under AddressSanitizer): new memory is filled with 0xBE,
//                       freed memory with 0xDD and kept out of reuse for a while (the quarantine, 128 MB): a read
//                       after free (a pointer into a destroyed temporary, an object a thread outlived) then fails at
//                       once, in the test that does it, instead of crashing a later test once in a while; a write
//                       after free, a double free or a foreign free is reported and fails the run.
//   EC_TESTS_THREADS=1  after each test, the threads it started and left running (given 2 s to end; the system's
//                       own thread pool aside) are reported, on Windows with the module each started in.
// Each run has its own temp directory (several ec_tests may run at once). A crash prints the test that was running
// (on Windows with the faulting address and the stack), not only the last test that finished.
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <new>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "harness.hpp"

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <tlhelp32.h>
#else
#include <unistd.h>
#if defined(__linux__)
#include <dirent.h>
#endif
#if defined(__GLIBC__)
#include <execinfo.h>
#endif
#endif

namespace {

// ============================================================================================ EC_TESTS_POISON

constexpr unsigned char kNewByte = 0xBE;
constexpr unsigned char kFreedByte = 0xDD;
constexpr std::uint64_t kLive = 0x11FEB10C11FEB10CULL;
constexpr std::uint64_t kFreed = 0xF4EEDB10CF4EEDB1ULL;
constexpr std::size_t kQuarantineBytes = std::size_t(128) << 20;
constexpr std::size_t kQuarantineBlocks = std::size_t(1) << 20;

// On unless EC_TESTS_POISON=0; off by default under AddressSanitizer, which finds the same and more.
#if defined(__SANITIZE_ADDRESS__)
constexpr int kPoisonByDefault = 0;
#elif defined(__has_feature)
#if __has_feature(address_sanitizer)
constexpr int kPoisonByDefault = 0;
#else
constexpr int kPoisonByDefault = 1;
#endif
#else
constexpr int kPoisonByDefault = 1;
#endif

/// In front of every block while poisoning (32 bytes: the block stays 16-byte aligned).
struct Header {
    std::uint64_t magic;
    std::uint64_t size;
    std::uint64_t pad[2];
};

int g_poison = -1;  // -1: not decided yet (the first allocation reads EC_TESTS_POISON), 0 off, 1 on
std::atomic_flag g_lock = ATOMIC_FLAG_INIT;  // a spin lock: usable before any constructor ran
Header** g_ring = nullptr;                   // the quarantine (g_lock), oldest at g_head
std::size_t g_head = 0, g_count = 0, g_bytes = 0;
std::atomic<long> g_memory_errors{0};

bool poisoning() {
    if (g_poison < 0) {
        const char* v = std::getenv("EC_TESTS_POISON");
        g_poison = v && *v ? (*v != '0' ? 1 : 0) : kPoisonByDefault;
    }
    return g_poison == 1;
}

struct SpinLock {
    SpinLock() {
        while (g_lock.test_and_set(std::memory_order_acquire)) std::this_thread::yield();
    }
    ~SpinLock() { g_lock.clear(std::memory_order_release); }
};

unsigned char* body(Header* h) { return reinterpret_cast<unsigned char*>(h) + sizeof(Header); }

/// The oldest block of the quarantine, taken out of it (g_lock held, g_count > 0).
Header* pop_oldest() {
    Header* oldest = g_ring[g_head];
    g_ring[g_head] = nullptr;
    g_head = (g_head + 1) % kQuarantineBlocks;
    --g_count;
    g_bytes -= oldest->size;
    return oldest;
}

void memory_error(const char* what, const Header* h, std::size_t offset) {
    if (g_memory_errors.fetch_add(1) >= 20) return;
    const char* test = ec_test::current().name;
    std::fprintf(stderr, "  MEMORY %s: a block of %llu bytes, byte %zu, seen during %s\n", what,
                 static_cast<unsigned long long>(h->size), offset, test ? test : "start-up");
    std::fflush(stderr);
}

void* allocate(std::size_t size) {
    if (!poisoning()) {
        if (void* p = std::malloc(size ? size : 1)) return p;
        throw std::bad_alloc();
    }
    auto* h = static_cast<Header*>(std::malloc(sizeof(Header) + (size ? size : 1)));
    if (!h) throw std::bad_alloc();
    h->magic = kLive;
    h->size = size;
    std::memset(body(h), kNewByte, size);
    return body(h);
}

void deallocate(void* p) noexcept {
    if (!p) return;
    if (!poisoning()) {
        std::free(p);
        return;
    }
    auto* h = reinterpret_cast<Header*>(static_cast<unsigned char*>(p) - sizeof(Header));
    if (h->magic != kLive) {
        memory_error(h->magic == kFreed ? "double free" : "free of a block never allocated here", h, 0);
        return;
    }
    h->magic = kFreed;
    std::memset(p, kFreedByte, h->size);
    Header* evicted[8];  // the oldest blocks leaving the quarantine (a few at most per free)
    std::size_t n = 0;
    {
        const SpinLock lock;
        if (!g_ring) g_ring = static_cast<Header**>(std::calloc(kQuarantineBlocks, sizeof(Header*)));
        if (!g_ring) {
            evicted[n++] = h;
        } else {
            if (g_count == kQuarantineBlocks) evicted[n++] = pop_oldest();
            g_ring[(g_head + g_count) % kQuarantineBlocks] = h;
            ++g_count;
            g_bytes += h->size;
            while (n < 8 && g_count > 1 && g_bytes > kQuarantineBytes) evicted[n++] = pop_oldest();
        }
    }
    for (std::size_t k = 0; k < n; ++k) {
        const unsigned char* b = body(evicted[k]);
        for (std::size_t i = 0; i < evicted[k]->size; ++i)
            if (b[i] != kFreedByte) {
                memory_error("write after free", evicted[k], i);
                break;
            }
        std::free(evicted[k]);
    }
}

// ============================================================================================ EC_TESTS_THREADS

/// The ids of this process's threads (empty: not known on this system).
std::set<unsigned long> live_threads() {
    std::set<unsigned long> ids;
#if defined(_WIN32)
    const HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    if (snap == INVALID_HANDLE_VALUE) return ids;
    const DWORD me = GetCurrentProcessId();
    THREADENTRY32 e;
    e.dwSize = sizeof e;
    for (BOOL ok = Thread32First(snap, &e); ok; ok = Thread32Next(snap, &e))
        if (e.th32OwnerProcessID == me) ids.insert(e.th32ThreadID);
    CloseHandle(snap);
#elif defined(__linux__)
    if (DIR* d = opendir("/proc/self/task")) {
        while (const dirent* e = readdir(d))
            if (e->d_name[0] != '.') ids.insert(std::strtoul(e->d_name, nullptr, 10));
        closedir(d);
    }
#endif
    return ids;
}

#if defined(_WIN32)
/// GetProcAddress as a typed function pointer (cast through void (*)(), which no compiler warns about).
template <class Fn>
Fn proc(HMODULE module, const char* name) {
    return module ? reinterpret_cast<Fn>(reinterpret_cast<void (*)()>(GetProcAddress(module, name))) : nullptr;
}

/// The file name of the module a thread started in: ntdll.dll for a worker of the system's thread pool, the C
/// runtime's DLL for a std::thread; "?" when not known.
std::string start_module(unsigned long id) {
    using QueryFn = LONG(WINAPI*)(HANDLE, int, PVOID, ULONG, PULONG);
    static const QueryFn query = proc<QueryFn>(GetModuleHandleA("ntdll.dll"), "NtQueryInformationThread");
    const HANDLE thread = query ? OpenThread(THREAD_QUERY_INFORMATION, FALSE, id) : nullptr;
    if (!thread) return "?";
    void* start = nullptr;
    const LONG status = query(thread, 9 /* ThreadQuerySetWin32StartAddress */, &start, sizeof start, nullptr);
    CloseHandle(thread);
    HMODULE module = nullptr;
    char path[MAX_PATH] = "?";
    if (status == 0 && start &&
        GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           static_cast<LPCSTR>(start), &module))
        GetModuleFileNameA(module, path, sizeof path);
    const char* slash = std::strrchr(path, '\\');
    return slash ? slash + 1 : path;
}
#endif

/// A worker of the system's own thread pool (Windows: started in ntdll), such as one SetupAPI's port listing woke: it
/// idles a while after its work and is never one of the driver's (the driver starts std::threads only).
bool system_pool_thread(unsigned long id) {
#if defined(_WIN32)
    return _stricmp(start_module(id).c_str(), "ntdll.dll") == 0;
#else
    (void)id;
    return false;
#endif
}

std::set<unsigned long> g_before;

void before_test(const char*) { g_before = live_threads(); }

/// The threads a test started and left running (the system's pool aside): given 2 s to end, then reported.
void after_test(const char* name) {
    std::vector<unsigned long> left;
    for (int i = 0; i <= 200; ++i) {
        left.clear();
        for (unsigned long id : live_threads())
            if (!g_before.count(id) && !system_pool_thread(id)) left.push_back(id);
        if (left.empty()) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::printf("  note %s left %zu thread(s) running:", name, left.size());
    for (unsigned long id : left) {
#if defined(_WIN32)
        std::printf(" %lu (%s)", id, start_module(id).c_str());
#else
        std::printf(" %lu", id);
#endif
    }
    std::printf("\n");
}

// ============================================================================================ a temp directory per run

/// Tests write files under fixed names in the temp directory, and several ec_tests may run at once (parallel builds of
/// one checkout): each run gets its own, <temp>/ec_tests_<pid>, as the process's temp directory (TMP and TEMP on
/// Windows, TMPDIR elsewhere: what std::filesystem::temp_directory_path() reads), removed at the end.
std::filesystem::path g_temp;

void use_own_temp_directory() {
    std::error_code ec;
    const std::filesystem::path base = std::filesystem::temp_directory_path(ec);
    if (ec) return;
#if defined(_WIN32)
    const std::filesystem::path own = base / ("ec_tests_" + std::to_string(GetCurrentProcessId()));
#else
    const std::filesystem::path own = base / ("ec_tests_" + std::to_string(getpid()));
#endif
    std::filesystem::remove_all(own, ec);  // left by an earlier run with this pid that crashed
    std::filesystem::create_directories(own, ec);
    if (ec) return;
#if defined(_WIN32)
    const std::wstring wide = own.wstring();
    if (!SetEnvironmentVariableW(L"TMP", wide.c_str()) || !SetEnvironmentVariableW(L"TEMP", wide.c_str())) return;
#else
    if (setenv("TMPDIR", own.c_str(), 1) != 0) return;
#endif
    g_temp = own;
}

void remove_own_temp_directory() {
    if (g_temp.empty()) return;
    std::error_code ec;
    std::filesystem::remove_all(g_temp, ec);
}

// ============================================================================================ crash report

void print_raw(const char* text) {
#if defined(_WIN32)
    std::fputs(text, stderr);
    std::fflush(stderr);
#else
    const ssize_t ignored = write(2, text, std::strlen(text));
    (void)ignored;
#endif
}

void print_crashed_test(const char* what) {
    const char* test = ec_test::current().name;
    print_raw("\n  CRASH (");
    print_raw(what);
    print_raw(") during ");
    print_raw(test ? test : "start-up");
    print_raw("\n");
}

extern "C" void on_abort(int) {
    print_crashed_test("abort");
    std::signal(SIGABRT, SIG_DFL);
    std::raise(SIGABRT);
}

#if defined(_WIN32)
// dbghelp, loaded only when a crash is reported (no link dependency)
struct SymbolInfo {
    ULONG SizeOfStruct, TypeIndex;
    ULONG64 Reserved[2];
    ULONG Index, Size;
    ULONG64 ModBase;
    ULONG Flags;
    ULONG64 Value, Address;
    ULONG Register, Scope, Tag, NameLen, MaxNameLen;
    CHAR Name[1];
};
struct LineInfo {
    DWORD SizeOfStruct;
    PVOID Key;
    DWORD LineNumber;
    PCHAR FileName;
    DWORD64 Address;
};
using SymInitializeFn = BOOL(WINAPI*)(HANDLE, PCSTR, BOOL);
using SymFromAddrFn = BOOL(WINAPI*)(HANDLE, DWORD64, PDWORD64, SymbolInfo*);
using SymGetLineFn = BOOL(WINAPI*)(HANDLE, DWORD64, PDWORD, LineInfo*);

LONG WINAPI on_unhandled(EXCEPTION_POINTERS* ep) {
    char text[160];
    const EXCEPTION_RECORD& r = *ep->ExceptionRecord;
    std::snprintf(text, sizeof text, "exception 0x%08lX at %p, thread %lu", static_cast<unsigned long>(r.ExceptionCode),
                  r.ExceptionAddress, static_cast<unsigned long>(GetCurrentThreadId()));
    print_crashed_test(text);
    if (r.ExceptionCode == EXCEPTION_ACCESS_VIOLATION && r.NumberParameters >= 2) {
        std::fprintf(stderr, "  %s of %p\n", r.ExceptionInformation[0] ? "write" : "read",
                     reinterpret_cast<void*>(r.ExceptionInformation[1]));
    }
    const HMODULE dbghelp = LoadLibraryA("dbghelp.dll");
    const auto init = proc<SymInitializeFn>(dbghelp, "SymInitialize");
    const auto symbol = proc<SymFromAddrFn>(dbghelp, "SymFromAddr");
    const auto line = proc<SymGetLineFn>(dbghelp, "SymGetLineFromAddr64");
    if (init) init(GetCurrentProcess(), nullptr, TRUE);
#if defined(_M_X64) || defined(__x86_64__)
    CONTEXT ctx = *ep->ContextRecord;
    for (int depth = 0; depth < 32 && ctx.Rip; ++depth) {
        alignas(8) char storage[sizeof(SymbolInfo) + 256] = {};
        auto* sym = reinterpret_cast<SymbolInfo*>(storage);
        sym->SizeOfStruct = 88;  // sizeof(SYMBOL_INFO)
        sym->MaxNameLen = 255;
        DWORD64 offset = 0;
        DWORD column = 0;
        LineInfo where{};
        where.SizeOfStruct = sizeof where;
        const bool named = symbol && symbol(GetCurrentProcess(), ctx.Rip, &offset, sym);
        const bool placed = line && line(GetCurrentProcess(), ctx.Rip, &column, &where);
        std::fprintf(stderr, "    #%d %p %s %s:%lu\n", depth, reinterpret_cast<void*>(ctx.Rip), named ? sym->Name : "?",
                     placed ? where.FileName : "?", placed ? static_cast<unsigned long>(where.LineNumber) : 0UL);
        DWORD64 image = 0;
        PRUNTIME_FUNCTION fn = RtlLookupFunctionEntry(ctx.Rip, &image, nullptr);
        if (fn) {
            PVOID data = nullptr;
            DWORD64 frame = 0;
            RtlVirtualUnwind(UNW_FLAG_NHANDLER, image, ctx.Rip, fn, &ctx, &data, &frame, nullptr);
        } else {  // a leaf function: the return address is on top of the stack
            ctx.Rip = *reinterpret_cast<DWORD64*>(ctx.Rsp);
            ctx.Rsp += 8;
        }
    }
#endif
    std::fflush(stderr);
    return EXCEPTION_CONTINUE_SEARCH;
}
#else
extern "C" void on_signal(int sig) {
    print_crashed_test(sig == SIGSEGV ? "SIGSEGV" : sig == SIGBUS ? "SIGBUS" : sig == SIGILL ? "SIGILL" : "SIGFPE");
#if defined(__GLIBC__)
    void* frames[32];
    backtrace_symbols_fd(frames, backtrace(frames, 32), 2);
#endif
    std::signal(sig, SIG_DFL);
    std::raise(sig);
}
#endif

void install_crash_report() {
    std::signal(SIGABRT, on_abort);
#if defined(_WIN32)
    SetUnhandledExceptionFilter(on_unhandled);
#else
    for (int sig : {SIGSEGV, SIGBUS, SIGILL, SIGFPE}) std::signal(sig, on_signal);
#endif
}

}  // namespace

// The global allocation functions of this executable (EC_TESTS_POISON; plain malloc / free otherwise).
void* operator new(std::size_t size) { return allocate(size); }
void* operator new[](std::size_t size) { return allocate(size); }
void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
    try {
        return allocate(size);
    } catch (...) {
        return nullptr;
    }
}
void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
    try {
        return allocate(size);
    } catch (...) {
        return nullptr;
    }
}
void operator delete(void* p) noexcept { deallocate(p); }
void operator delete[](void* p) noexcept { deallocate(p); }
void operator delete(void* p, std::size_t) noexcept { deallocate(p); }
void operator delete[](void* p, std::size_t) noexcept { deallocate(p); }
void operator delete(void* p, const std::nothrow_t&) noexcept { deallocate(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { deallocate(p); }

int main(int argc, char** argv) {
    install_crash_report();
    const char* threads = std::getenv("EC_TESTS_THREADS");
    if (threads && *threads && *threads != '0' && !live_threads().empty()) {
        ec_test::before_each() = &before_test;
        ec_test::after_each() = &after_test;
    }
    use_own_temp_directory();
    const int code = ec_test::main_entry(argc, argv);
    remove_own_temp_directory();
    if (g_memory_errors.load() > 0) {
        std::printf("%ld memory error(s) reported (EC_TESTS_POISON)\n", g_memory_errors.load());
        return 1;
    }
    return code;
}
