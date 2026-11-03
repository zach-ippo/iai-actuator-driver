/*
 * first_move.c - one axis through the C ABI (include/iai_actuator_driver.h): open, connect, servo ON, home when
 * needed, an absolute move and two relative moves around mid-stroke, the status, servo OFF, close.
 *
 * Run it (close IA-OS first: only one program can use a serial port):
 *
 *     first_move --sim                       the simulator (an ELECYLINDER slider), no hardware
 *     first_move --sim --sim-profile rod     another simulated unit: slider, gripper, rod, pcon, rotary, unknown
 *     first_move --port COM3                 the unit on COM3, address 1 (the baud rate is found automatically)
 *     first_move --port COM3 --slave 2       the unit at address 2 on a shared port (address = axis number + 1)
 *     first_move --port COM3 --home          a home return even when the status says homed
 *
 * Options: --speed MM_S (default 20, raised to the unit's documented minimum speed where that is higher),
 * --distance MM (the step out and back, default 10, at most a quarter of the stroke).
 *
 * Every target and speed is checked against the unit's own ranges before anything is sent: a value outside them gives
 * IAD_ERR_SOFT_LIMIT and nothing moves. Every function returns an iad_result; on an error the calling thread's
 * iad_last_error_message() says what happened, and iad_last_error_json() has the details (alarm code, reasons).
 *
 * Motion: to the middle of the stroke, then --distance further and back. The home return runs only when the status
 * says the axis is not homed, or with --home: the axis runs to its mechanical end, so keep the travel clear. Test a
 * new setup unloaded, and keep a hand on the emergency stop.
 *
 * End: the servo is switched OFF (a vertical axis without a brake can fall when the servo goes off: support it first)
 * and Modbus control is handed back to PIO when PIO had it at the start. Ctrl+C sends a decel STOP from the handler's
 * thread (iad_axis_stop may be called while another thread waits for a move); the waiting move then returns an error
 * and the program closes the port with the servo left as it is. Exit codes: 0 done, 2 error, 130 interrupted.
 *
 * Build (any C99 compiler; the DLL next to the program at run time):
 *
 *     zig cc -std=c99 -I core/include examples/c/first_move.c core/build/iai_actuator_driver.lib -o first_move.exe
 *     cc -std=c99 -I core/include examples/c/first_move.c -L core/build -liai_actuator_driver -lpthread -lm (Linux)
 *
 * Unofficial: not an IAI product. The driver is not a safety function: keep a hardware emergency stop.
 */
#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200809L /* sigwait, pthread_sigmask */
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "iai_actuator_driver.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#include <signal.h>
#include <time.h>
#endif

#define BANNER "iai-actuator-driver example: first move, C ABI (UNOFFICIAL - not an IAI product)"
#define DEFAULT_SPEED 20.0    /* units/s: the speed for a first test (unloaded, mid-stroke, 20 mm/s or less) */
#define DEFAULT_DISTANCE 10.0 /* units: the step out and back */

/* ------------------------------------------------------------------------------------------- Ctrl+C */

/* The axis the Ctrl+C handler stops. A closed handle is safe to pass: the call returns IAD_ERR_INVALID_ARGUMENT. */
static iad_axis* volatile g_axis = NULL;
static volatile int g_interrupted = 0; /* Ctrl+C was pressed */
static volatile int g_stop_done = 0;   /* the handler's STOP returned (the axis is at a standstill or it failed) */

/* Runs on its own thread (never in a signal handler): a decel STOP, after which a move waiting in main() returns an
 * error. Returns 0 on a second Ctrl+C, which then ends the program at once. */
static int stop_on_interrupt(void) {
    if (g_interrupted) {
        return 0;
    }
    g_interrupted = 1;
    if (g_axis != NULL) {
        iad_axis_stop(g_axis, NULL, NULL); /* NULL options: the defaults; blocks for the deceleration */
    }
    g_stop_done = 1;
    return 1;
}

#ifdef _WIN32
/* Windows calls a console control handler on a new thread. */
static BOOL WINAPI on_console_event(DWORD event) {
    if (event != CTRL_C_EVENT && event != CTRL_BREAK_EVENT) {
        return FALSE;
    }
    return stop_on_interrupt() ? TRUE : FALSE; /* TRUE: handled, main() reports it and closes the port */
}

static void install_interrupt_handler(void) {
    SetConsoleCtrlHandler(on_console_event, TRUE);
}

static void sleep_ms(int ms) {
    Sleep((DWORD)ms);
}
#else
static sigset_t g_signals;

/* A thread that waits for SIGINT / SIGTERM; the other threads block them (the mask is inherited, so this runs before
 * the driver starts any thread). */
static void* interrupt_thread(void* unused) {
    int sig = 0;
    (void)unused;
    while (sigwait(&g_signals, &sig) == 0) {
        if (!stop_on_interrupt()) {
            _Exit(130);
        }
    }
    return NULL;
}

static void install_interrupt_handler(void) {
    pthread_t thread;
    sigemptyset(&g_signals);
    sigaddset(&g_signals, SIGINT);
    sigaddset(&g_signals, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &g_signals, NULL);
    if (pthread_create(&thread, NULL, interrupt_thread, NULL) == 0) {
        pthread_detach(thread);
    }
}

static void sleep_ms(int ms) {
    struct timespec t;
    t.tv_sec = ms / 1000;
    t.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&t, NULL);
}
#endif

/* ------------------------------------------------------------------------------------------- command line */

typedef struct options {
    const char* port;        /* --port COM3 */
    const char* sim_profile; /* --sim: "slider" unless --sim-profile names another */
    int slave;
    double speed;            /* NaN = DEFAULT_SPEED or the documented minimum */
    double distance;
    int home;
} options;

static int usage(const char* problem) {
    if (problem != NULL) {
        fprintf(stderr, "error: %s\n", problem);
    }
    fprintf(stderr,
            "usage: first_move (--sim [--sim-profile NAME] | --port PORT) [--slave N] [--speed UNITS_S]\n"
            "                  [--distance UNITS] [--home]\n");
    return 2;
}

static int positive(const char* text, double* out) {
    char* end = NULL;
    const double value = strtod(text, &end);
    if (end == text || *end != '\0' || !(value > 0.0) || !isfinite(value)) {
        return 0;
    }
    *out = value;
    return 1;
}

static int parse_args(int argc, char** argv, options* o) {
    int i;
    int sim = 0;
    o->port = NULL;
    o->sim_profile = "slider";
    o->slave = 1;
    o->speed = NAN;
    o->distance = DEFAULT_DISTANCE;
    o->home = 0;
    for (i = 1; i < argc; ++i) {
        const char* a = argv[i];
        const char* value = (i + 1 < argc) ? argv[i + 1] : NULL;
        if (strcmp(a, "--sim") == 0) {
            sim = 1;
        } else if (strcmp(a, "--home") == 0) {
            o->home = 1;
        } else if (strcmp(a, "-h") == 0 || strcmp(a, "--help") == 0) {
            usage(NULL);
            exit(0);
        } else if (value == NULL) {
            return usage("an option is missing its value, or is unknown");
        } else if (strcmp(a, "--port") == 0) {
            o->port = value;
            ++i;
        } else if (strcmp(a, "--sim-profile") == 0) {
            o->sim_profile = value;
            ++i;
        } else if (strcmp(a, "--slave") == 0) {
            o->slave = atoi(value);
            ++i;
            if (o->slave < 1 || o->slave > 247) {
                return usage("--slave takes 1..247");
            }
        } else if (strcmp(a, "--speed") == 0) {
            if (!positive(value, &o->speed)) {
                return usage("--speed takes a number above 0");
            }
            ++i;
        } else if (strcmp(a, "--distance") == 0) {
            if (!positive(value, &o->distance)) {
                return usage("--distance takes a number above 0");
            }
            ++i;
        } else {
            return usage("unknown option");
        }
    }
    if (sim == (o->port != NULL)) {
        return usage("give --sim or --port PORT (one of them)");
    }
    if (sim) {
        o->port = NULL;
    }
    return 0;
}

/* ------------------------------------------------------------------------------------------- output */

static void show(const char* label, const char* text) {
    printf("%-10s %s\n", label, text);
    fflush(stdout);
}

/* The status in one line: position and the flags that matter around a move. */
static void show_status(const char* label, const iad_status* st, const char* units) {
    char text[256];
    int n = snprintf(text, sizeof text, "position %.2f %s,", st->position_mm, units);
    if (st->servo_on) n += snprintf(text + n, sizeof text - (size_t)n, " servo ON,");
    if (st->homed) n += snprintf(text + n, sizeof text - (size_t)n, " homed,");
    if (st->in_position) n += snprintf(text + n, sizeof text - (size_t)n, " in position,");
    if (st->moving) n += snprintf(text + n, sizeof text - (size_t)n, " moving,");
    if (st->modbus_enabled) n += snprintf(text + n, sizeof text - (size_t)n, " PC control,");
    if (st->emergency_stop) n += snprintf(text + n, sizeof text - (size_t)n, " emergency stop,");
    if (st->alarm || st->warning) {
        n += snprintf(text + n, sizeof text - (size_t)n, " %s 0x%03X,", st->alarm ? "alarm" : "warning",
                      (unsigned)st->alarm_code);
    }
    text[n - 1] = '\0'; /* drop the last comma */
    show(label, text);
}

/* Print a failed call: the result name and this thread's last error message; the JSON details for the errors that
 * carry more (the alarm code, the reasons a servo cannot come on). */
static void report_error(const char* call, iad_result rc) {
    static char details[4096];
    size_t need = 0;
    fflush(stdout);
    fprintf(stderr, "error: %s: %s: %s\n", call, iad_result_name(rc), iad_last_error_message());
    if ((rc == IAD_ERR_ALARM || rc == IAD_ERR_SERVO_BLOCKED) &&
        iad_last_error_json(details, sizeof details, &need) == IAD_OK) {
        fprintf(stderr, "       details: %s\n", details);
    }
}

/* Every driver call goes through CALL: on an error (or after Ctrl+C) it jumps to the end of main. */
#define CALL(expr)                                \
    do {                                          \
        const iad_result rc_ = (expr);            \
        if (rc_ != IAD_OK) {                      \
            if (!g_interrupted) report_error(#expr, rc_); \
            goto end;                             \
        }                                         \
        if (g_interrupted) goto end;              \
    } while (0)

/* ------------------------------------------------------------------------------------------- main */

int main(int argc, char** argv) {
    options args;
    iad_sim_options sim;
    iad_open_options open;
    iad_move_options move;
    iad_home_options home;
    iad_identity id;
    iad_limits lim;
    iad_status found;
    iad_status st;
    iad_axis* ax = NULL;
    const uint32_t abi = iad_abi_version();
    double low, high, middle, step, speed;
    int motion = 0; /* a motion command ran: an error afterwards sends a decel STOP before the port closes */
    int exit_code = 2;
    char text[256];
    int rc = parse_args(argc, argv, &args);
    if (rc != 0) {
        return rc;
    }
    puts(BANNER);
    /* The loader rule before any other call: the same ABI major and a library minor at least the one this program was
     * compiled against (a higher minor only adds). */
    if (!IAD_ABI_COMPATIBLE(abi)) {
        fprintf(stderr, "error: the library has ABI %u.%u, this program was built for %d.%d\n", (unsigned)(abi >> 16),
                (unsigned)(abi & 0xFFFFu), IAD_ABI_VERSION_MAJOR, IAD_ABI_VERSION_MINOR);
        return 2;
    }
    install_interrupt_handler();

    /* Open: a simulated unit, or the port (the baud rate is found in place). Option structs start from their
     * defaults (iad_*_options_init), never from zero; struct_size = sizeof is set before each _init (the sized-struct
     * rule: the library writes only as much as the caller's struct holds). */
    open.struct_size = sizeof open;
    CALL(iad_open_options_init(&open));
    open.slave = args.slave;
    if (args.port == NULL) {
        sim.struct_size = sizeof sim;
        CALL(iad_sim_options_init(&sim));
        sim.pmsl = 0; /* PIO in control, as after a power-up: connect takes Modbus control as on hardware */
        CALL(iad_axis_open_sim(args.sim_profile, &sim, &open, NULL, &ax));
    } else {
        CALL(iad_axis_open(args.port, &open, NULL, &ax));
    }
    g_axis = ax;

    CALL(iad_axis_status(ax, &found));      /* read-only: the state before this program sent anything */
    CALL(iad_axis_connect(ax, 1, &id));     /* identity and ranges; Modbus control is taken here, at rest only */
    CALL(iad_axis_limits(ax, &lim));        /* every range the driver enforces, read from the unit itself */
    snprintf(text, sizeof text, "%s (%s; actuator: %s), firmware %s, address %d, library %s", id.model,
             id.product_family, id.kind, id.firmware, id.slave, iad_library_version());
    show("unit", text);
    low = lim.position.min;
    high = lim.position.max;
    if (isnan(low) || isnan(high)) {
        fprintf(stderr, "error: the unit's soft limits are not readable here: set stroke_min / stroke_max in "
                        "iad_axis_options\n");
        goto end;
    }
    speed = args.speed;
    if (isnan(speed)) {
        speed = DEFAULT_SPEED;
        if (!isnan(lim.speed.recommended_min) && lim.speed.recommended_min > speed) {
            speed = lim.speed.recommended_min; /* below it a unit may run noisily; the driver only warns */
        }
    }
    snprintf(text, sizeof text, "%.2f .. %.2f %s; speed up to %g %s/s; %g %s/s for every move here", low, high,
             id.units, lim.speed.max, id.units, speed, id.units);
    show("range", text);
    show_status("status", &found, id.units);

    /* A servo ON the status shows cannot work (EMG, drive power cut) is refused with IAD_ERR_SERVO_BLOCKED. */
    CALL(iad_axis_servo(ax, 1, NULL, &st)); /* NULL options: the defaults (wait, the core's timeout) */
    show("servo", "ON");
    if (args.home || !st.homed) {
        show("home", "home return: the axis runs to its mechanical end, keep the travel clear");
        home.struct_size = sizeof home;
        CALL(iad_home_options_init(&home)); /* every axis command takes an options struct; NULL = these defaults */
        home.wait = 1;                       /* wait for HEND and standstill (the default) */
        CALL(iad_axis_home(ax, &home, &st));
        motion = 1;
        snprintf(text, sizeof text, "homed at %.2f %s", st.position_mm, id.units);
        show("", text);
    } else {
        show("home", "not needed: the status says homed (--home runs a home return anyway)");
    }

    middle = floor((low + high) / 2.0 * 100.0 + 0.5) / 100.0;
    step = fmin(args.distance, (high - low) / 4.0);
    step = floor(step * 100.0) / 100.0;
    move.struct_size = sizeof move;
    CALL(iad_move_options_init(&move));
    move.speed = speed; /* accel 0.1 G, band 0.1, wait until in position and settled: the defaults */
    CALL(iad_axis_move_to(ax, middle, &move, &st));
    motion = 1;
    snprintf(text, sizeof text, "to the middle of the stroke: at %.2f %s", st.position_mm, id.units);
    show("move", text);
    CALL(iad_axis_move_by(ax, step, &move, &st)); /* relative to the measured position */
    snprintf(text, sizeof text, "by %+.2f %s: at %.2f %s", step, id.units, st.position_mm, id.units);
    show("move", text);
    CALL(iad_axis_move_by(ax, -step, &move, &st));
    snprintf(text, sizeof text, "by %+.2f %s: at %.2f %s", -step, id.units, st.position_mm, id.units);
    show("move", text);
    motion = 0;

    CALL(iad_axis_status(ax, &st));
    show_status("status", &st, id.units);
    CALL(iad_axis_servo(ax, 0, NULL, NULL));
    show("servo", "OFF");
    if (!found.modbus_enabled) {
        CALL(iad_axis_modbus_control(ax, 0)); /* PIO had control when this program started: hand it back */
        show("control", "handed back to PIO");
    }
    exit_code = 0;

end:
    if (g_interrupted) {
        while (!g_stop_done) {
            sleep_ms(20); /* the handler's STOP holds until standstill: close the port after it */
        }
        fflush(stdout);
        fprintf(stderr, "interrupted (Ctrl+C): a decel STOP was sent; the servo is left as it is\n");
        exit_code = 130;
    } else if (exit_code != 0 && motion && ax != NULL) {
        iad_axis_stop(ax, NULL, NULL); /* an error after a motion command: decel STOP before the port closes */
        fprintf(stderr, "       a decel STOP was sent; the servo is left as it is\n");
    }
    g_axis = NULL;
    iad_axis_close(ax); /* NULL or closed: IAD_OK */
    if (exit_code == 0) {
        puts("done");
    }
    return exit_code;
}
