/*
 * C smoke test of the C ABI (include/iai_actuator_driver.h). Compiled as plain C99 with -pedantic against the shared
 * library's import library and run next to the DLL (python build.py smoke; CMake: the iad_c_smoke test), so it proves
 * that the header is valid C and that a C program drives the library through its exports: versions, option defaults,
 * an axis and a cell on the simulator, the JSON size query, error codes and messages, closing twice and NULL handles,
 * and (ABI 0.2) a move handle, the status monitor with its event queue, unit info, the cancel entry, per-axis move
 * values and pause / resume on a cell. Simulator only: no COM port is opened.
 *
 * Evidence: UNVERIFIED on hardware (the simulator stands in for the units).
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "iai_actuator_driver.h"

static int g_checks = 0;
static int g_failed = 0;

static void expect(int ok, const char* text, int line) {
    ++g_checks;
    if (!ok) {
        ++g_failed;
        fprintf(stderr, "FAIL c_smoke.c:%d: %s (last error: %s)\n", line, text, iad_last_error_message());
    }
}

static void expect_rc(iad_result got, iad_result want, const char* text, int line) {
    ++g_checks;
    if (got != want) {
        ++g_failed;
        fprintf(stderr, "FAIL c_smoke.c:%d: %s gave %s, expected %s (%s)\n", line, text, iad_result_name(got),
                iad_result_name(want), iad_last_error_message());
    }
}

#define EXPECT(cond) expect((cond) ? 1 : 0, #cond, __LINE__)
#define EXPECT_RC(call, want) expect_rc((call), (want), #call, __LINE__)

int main(void) {
    static char buf[65536];
    const uint32_t abi = iad_abi_version();
    iad_sim_options sim;
    iad_move_options move;
    iad_servo_options servo;
    iad_home_options home;
    iad_wait_idle_options wait_idle;
    iad_identity id;
    iad_status st;
    iad_limits lim;
    iad_axis* ax = NULL;
    iad_cell* cell = NULL;
    iad_cell_result results[2];
    iad_axis_move each[2];
    double targets[2];
    double decels[2];
    iad_move_handle* mh = NULL;
    iad_move_info info;
    iad_monitor_options mon;
    iad_event ev;
    iad_motion_caps caps;
    iad_bool done = 0;
    int got_done = 0;
    int polls = 0;
    size_t need = 0;
    size_t count = 0;

    /* the library and its version: the loader rule before any other call (IAD_ABI_COMPATIBLE: the same major and a
     * library minor at least the header's), then this build's own */
    if (!IAD_ABI_COMPATIBLE(abi)) {
        fprintf(stderr, "the library has ABI %u.%u; this program was built for %d.%d\n", (unsigned)(abi >> 16),
                (unsigned)(abi & 0xFFFFu), IAD_ABI_VERSION_MAJOR, IAD_ABI_VERSION_MINOR);
        return 1;
    }
    EXPECT(abi == (uint32_t)IAD_ABI_VERSION);
    EXPECT(abi == (((uint32_t)IAD_ABI_VERSION_MAJOR << 16) | (uint32_t)IAD_ABI_VERSION_MINOR));
    printf("abi %u.%u, library %s\n", (unsigned)(abi >> 16), (unsigned)(abi & 0xFFFFu), iad_library_version());
    EXPECT(strcmp(iad_result_name(IAD_ERR_CELL), "IAD_ERR_CELL") == 0);
    EXPECT(strcmp(iad_result_name(-1000), "IAD_ERR_UNKNOWN") == 0);
    EXPECT_RC(iad_move_options_init(NULL), IAD_ERR_INVALID_ARGUMENT);
    EXPECT(strlen(iad_last_error_message()) > 0);

    /* one axis on a simulated slider */
    sim.struct_size = sizeof sim; /* the sized-struct rule: set before _init */
    EXPECT_RC(iad_sim_options_init(&sim), IAD_OK);
    sim.time_scale = 20.0;
    EXPECT_RC(iad_axis_open_sim("slider", &sim, NULL, NULL, &ax), IAD_OK);
    EXPECT_RC(iad_axis_connect(ax, 1, &id), IAD_OK);
    EXPECT(strcmp(id.product_family, "EC") == 0);
    EXPECT(strcmp(id.units, "mm") == 0);
    EXPECT_RC(iad_axis_servo(ax, 1, NULL, NULL), IAD_OK); /* NULL options: wait, the core's timeout */
    home.struct_size = sizeof home;
    EXPECT_RC(iad_home_options_init(&home), IAD_OK);      /* 0.8: an options struct, as every axis command */
    EXPECT(home.struct_size == (uint32_t)sizeof home && home.wait == IAD_TRUE && home.method == IAD_METHOD_DEFAULT);
    EXPECT_RC(iad_axis_home(ax, &home, &st), IAD_OK);
    EXPECT(st.homed == IAD_TRUE);
    servo.struct_size = sizeof servo;
    EXPECT_RC(iad_servo_options_init(&servo), IAD_OK);
    servo.timeout_s = 5.0;
    EXPECT_RC(iad_axis_servo(ax, 1, &servo, &st), IAD_OK);
    EXPECT(st.servo_on == IAD_TRUE);
    move.struct_size = sizeof move;
    EXPECT_RC(iad_move_options_init(&move), IAD_OK);
    EXPECT(move.struct_size == (uint32_t)sizeof move && move.reserved_header == 0); /* 0.8: a sized input */
    move.speed = 100.0;
    EXPECT_RC(iad_axis_move_to(ax, 50.0, &move, &st), IAD_OK);
    EXPECT(fabs(st.position_mm - 50.0) < 0.11);
    /* 0.8, the rule of the defaults: IAD_UNKNOWN is 2 (an input tribool: 0 false, 2 keep, any other value true, a
     * Visual Basic True -1 too) and 0 is the default of every enumeration */
    EXPECT(IAD_FALSE == 0 && IAD_TRUE == 1 && IAD_UNKNOWN == 2);
    EXPECT(IAD_METHOD_DEFAULT == 0 && IAD_METHOD_COIL == 1 && IAD_METHOD_IAI == 2 && IAD_METHOD_REGISTER == 4);
    EXPECT(move.accel_pattern == IAD_ACCEL_DEFAULT && move.completion == IAD_COMPLETION_DEFAULT);
    EXPECT_RC(iad_axis_set_action_mode(ax, IAD_TRUE, -1, IAD_TRUE, &st), IAD_OK);
    EXPECT(st.safety_speed == IAD_TRUE);
    EXPECT_RC(iad_axis_set_action_mode(ax, IAD_TRUE, IAD_UNKNOWN, IAD_TRUE, &st), IAD_OK);
    EXPECT(st.safety_speed == IAD_TRUE);
    EXPECT_RC(iad_axis_set_action_mode(ax, IAD_TRUE, IAD_FALSE, IAD_TRUE, &st), IAD_OK);
    EXPECT(st.safety_speed == IAD_FALSE);
    EXPECT_RC(iad_axis_move_to(ax, 1e6, &move, &st), IAD_ERR_SOFT_LIMIT);
    EXPECT(strlen(iad_last_error_message()) > 0);
    EXPECT_RC(iad_last_error_json(buf, sizeof buf, &need), IAD_OK);
    EXPECT(strstr(buf, "\"kind\":\"SoftLimitError\"") != NULL);
    EXPECT_RC(iad_axis_limits(ax, &lim), IAD_OK);
    EXPECT(strcmp(lim.units, "mm") == 0 && lim.position.max > 100.0);

    /* JSON: the size query, then the text */
    EXPECT_RC(iad_axis_limits_json(ax, NULL, 0, &need), IAD_ERR_BUFFER_TOO_SMALL);
    EXPECT(need > 2 && need <= sizeof buf);
    EXPECT_RC(iad_axis_limits_json(ax, buf, need, &need), IAD_OK);
    EXPECT(buf[0] == '{' && strlen(buf) + 1 == need);
    EXPECT_RC(iad_last_error_json(buf, sizeof buf, &need), IAD_OK);
    EXPECT(strcmp(buf, "{\"code\":0}") == 0);

    /* 0.2: a move handle, the status monitor and its event queue (polled with a timeout, no callback) */
    EXPECT(strcmp(iad_result_name(IAD_ERR_CANCELLED), "IAD_ERR_CANCELLED") == 0);
    EXPECT_RC(iad_axis_start_move_to(ax, 60.0, &move, &mh), IAD_OK);
    EXPECT_RC(iad_move_handle_wait(mh, NAN, &st), IAD_OK);
    EXPECT(fabs(st.position_mm - 60.0) < 0.11);
    EXPECT_RC(iad_move_handle_done(mh, &done, NULL), IAD_OK);
    EXPECT(done == IAD_TRUE);
    EXPECT_RC(iad_move_handle_result(mh, &info), IAD_OK);
    EXPECT(info.has_result == IAD_TRUE && fabs(info.target - 60.0) < 1e-9);
    EXPECT_RC(iad_move_handle_close(mh), IAD_OK);
    EXPECT_RC(iad_move_handle_close(mh), IAD_OK);
    mon.struct_size = sizeof mon;
    EXPECT_RC(iad_monitor_options_init(&mon), IAD_OK);
    mon.events = IAD_EVENTS_DONE;
    EXPECT_RC(iad_axis_start_monitor(ax, &mon), IAD_OK);
    EXPECT_RC(iad_axis_move_to(ax, 40.0, &move, &st), IAD_OK);
    while (!got_done && polls++ < 100) { /* at most 100 x 0.1 s */
        EXPECT_RC(iad_axis_next_event(ax, 0.1, &ev), IAD_OK);
        got_done = ev.kind == IAD_EVENT_DONE;
    }
    EXPECT(got_done && fabs(ev.status.position_mm - 40.0) < 0.11 && ev.sequence >= 1);
    EXPECT(ev.index == IAD_INT_UNKNOWN); /* 0.8: an axis's own queue has no cell position */
    EXPECT_RC(iad_axis_stop_monitor(ax, NAN), IAD_OK);
    wait_idle.struct_size = sizeof wait_idle;
    EXPECT_RC(iad_wait_idle_options_init(&wait_idle), IAD_OK);
    wait_idle.target = 40.0;
    wait_idle.completion = IAD_COMPLETION_FAST;
    EXPECT_RC(iad_axis_wait_idle(ax, &wait_idle, &st), IAD_OK);
    EXPECT_RC(iad_axis_read_lean_status(ax, IAD_INT_UNKNOWN, &st), IAD_OK);
    EXPECT(st.raw_words == 8 && fabs(st.position_mm - 40.0) < 0.11);
    EXPECT_RC(iad_axis_cancel(ax), IAD_OK); /* nothing blocking: no effect */
    EXPECT_RC(iad_axis_read_unit_info_json(ax, "identity", NULL, 0, &need), IAD_ERR_BUFFER_TOO_SMALL);
    EXPECT(need > 2 && need <= sizeof buf);
    EXPECT_RC(iad_axis_last_result_json(ax, buf, sizeof buf, &need), IAD_OK); /* kept: no second read */
    EXPECT(strstr(buf, "\"identity\"") != NULL);
    EXPECT_RC(iad_axis_motion_caps(ax, &caps), IAD_OK);
    EXPECT(caps.speed_cap > 0.0 && caps.accel_cap > 0.0 && caps.push_cap > 0.0);

    /* closing twice and NULL are safe; a closed handle is refused */
    EXPECT_RC(iad_axis_close(ax), IAD_OK);
    EXPECT_RC(iad_axis_close(ax), IAD_OK);
    EXPECT_RC(iad_axis_close(NULL), IAD_OK);
    EXPECT_RC(iad_axis_status(ax, &st), IAD_ERR_INVALID_ARGUMENT);
    EXPECT_RC(iad_axis_status(NULL, &st), IAD_ERR_INVALID_ARGUMENT);

    /* a cell of two simulated lines */
    EXPECT_RC(iad_cell_open_sim("slider,gripper", 1, &sim, NULL, NULL, &cell), IAD_OK);
    EXPECT_RC(iad_cell_axis_count(cell, &count), IAD_OK);
    EXPECT(count == 2);
    EXPECT_RC(iad_cell_axis_name(cell, 1, buf, sizeof buf, &need), IAD_OK);
    EXPECT(strcmp(buf, "SIM2:1") == 0);
    EXPECT_RC(iad_cell_servo_all(cell, 1, NULL, NAN, results, 2, &count), IAD_OK);
    EXPECT_RC(iad_cell_home_all(cell, NULL, NAN, IAD_METHOD_DEFAULT, IAD_TRUE, results, 2, &count), IAD_OK);
    move.speed = 50.0; /* within the gripper's 53 mm/s */
    targets[0] = 30.0;
    targets[1] = 5.0;
    EXPECT_RC(iad_cell_move_all(cell, "SIM1:1,SIM2:1", targets, 2, &move, NULL, NULL, NULL, NULL, results, 2, &count),
              IAD_OK);
    EXPECT(count == 2 && fabs(results[1].status.position_mm - 5.0) < 0.11);
    EXPECT(strcmp(results[0].axis, "SIM1:1") == 0 && results[0].code == IAD_OK);
    EXPECT(results[0].index == 0 && results[1].index == 1); /* 0.8: the position in the request */
    targets[1] = 500.0; /* outside the gripper's stroke: the whole move is refused */
    EXPECT_RC(iad_cell_move_all(cell, "SIM1:1,SIM2:1", targets, 2, &move, NULL, NULL, NULL, NULL, results, 2, &count),
              IAD_ERR_CELL);
    EXPECT(count == 2 && results[1].code == IAD_ERR_SOFT_LIMIT);
    targets[1] = 2.0; /* 0.2: per-axis values: the gripper decelerates at a value of its own (command profile) */
    decels[0] = NAN;
    decels[1] = 0.05;
    EXPECT_RC(iad_cell_move_all(cell, "SIM1:1,SIM2:1", targets, 2, &move, NULL, NULL, NULL, decels, results, 2, &count),
              IAD_OK);
    EXPECT(count == 2 && fabs(results[1].status.position_mm - 2.0) < 0.11);
    /* 0.8: an iad_axis_move array: zeroed (NaN or 0 = the defaults), struct_size = the stride */
    memset(each, 0, sizeof each);
    each[0].struct_size = each[1].struct_size = (uint32_t)sizeof each[0];
    each[0].position = 35.0;
    strcpy(each[0].axis, "SIM1:1");
    each[1].position = 3.0;
    each[1].speed = 20.0;
    strcpy(each[1].axis, "SIM2:1");
    EXPECT_RC(iad_cell_move_each(cell, each, 2, 1, NAN, IAD_TRUE, results, 2, &count), IAD_OK);
    EXPECT(count == 2 && results[1].index == 1 && fabs(results[1].status.position_mm - 3.0) < 0.11);
    /* 0.2: coordinated motion: a JSON request, a dry run, then the run (its result kept on the handle) */
    EXPECT_RC(iad_cell_coord_plan_json(cell,
                                       "{\"axes\": [\"SIM1:1\", \"SIM2:1\"], \"speed\": 10,"
                                       " \"path\": {\"segments\": [{\"line\": {\"SIM1:1\": 40, \"SIM2:1\": 4}}]}}",
                                       buf, sizeof buf, &need),
              IAD_OK);
    EXPECT(strstr(buf, "\"mode\":\"sync\"") != NULL);
    EXPECT_RC(iad_cell_coord_run_json(cell,
                                      "{\"axes\": [\"SIM1:1\", \"SIM2:1\"], \"speed\": 10,"
                                      " \"path\": {\"segments\": [{\"line\": {\"SIM1:1\": 40, \"SIM2:1\": 4}}]}}",
                                      NULL, 0, &need),
              IAD_ERR_BUFFER_TOO_SMALL);
    EXPECT_RC(iad_cell_last_result_json(cell, buf, sizeof buf, &need), IAD_OK);
    EXPECT(strstr(buf, "\"completed\":true") != NULL);
    EXPECT_RC(iad_cell_pause_all(cell, NULL, 0, results, 2, &count), IAD_OK);
    EXPECT_RC(iad_cell_resume_all(cell, NULL, 0, results, 2, &count), IAD_OK);
    EXPECT(count == 2);
    EXPECT_RC(iad_cell_stop_all(cell, NULL, NAN, IAD_FALSE, results, 1, &count), IAD_OK); /* too small: all handled */
    EXPECT(count == 2);
    EXPECT_RC(iad_cell_close(cell), IAD_OK);
    EXPECT_RC(iad_cell_close(cell), IAD_OK);
    EXPECT_RC(iad_cell_axis_count(cell, &count), IAD_ERR_INVALID_ARGUMENT);

    printf("c_smoke: %d checks, %d failed\n", g_checks, g_failed);
    return g_failed ? 1 : 0;
}
