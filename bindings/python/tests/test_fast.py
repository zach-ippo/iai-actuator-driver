"""Fast completion mode (simulator only, no serial port): the status monitor, completion="fast", the 1 ms timer,
the pre-move read skip, MoveHandle, blended sequences and Axis.open(fast=True)."""

from __future__ import annotations

import json
import math
import threading
import time
import unittest

from ec_testutil import FrameRecorder, SteppedClockSim, make_ec

from iai_actuator_driver import monitor as M
from iai_actuator_driver import registers as R
from iai_actuator_driver.ec import Axis, Elecylinder, MoveHandle, Status
from iai_actuator_driver.errors import EcAlarmError, ModbusError, MotionTimeoutError, SoftLimitError
from iai_actuator_driver.modbus import ModbusRTU
from iai_actuator_driver.monitor import AxisMonitor, fast_done
from iai_actuator_driver.sim import SimulatedEC
from iai_actuator_driver.sim_line import sim_line
from iai_actuator_driver.testing import virtual_clock


def is_status_read(frame: bytes, words: int | None = None) -> bool:
    if frame[1] != 0x03 or int.from_bytes(frame[2:4], "big") != R.MONITOR_BASE:
        return False
    return words is None or int.from_bytes(frame[4:6], "big") == words


def is_move_write(frame: bytes) -> bool:
    return frame[1] == 0x10 and int.from_bytes(frame[2:4], "big") == R.DIRECT_MOVE_BASE


def is_stop_on(frame: bytes) -> bool:
    return frame[1] == 0x05 and int.from_bytes(frame[2:4], "big") == R.COIL_STOP and frame[4] == 0xFF


class ThreadRecorder(FrameRecorder):
    """Frame logger that also notes the sending thread, the time and the simulator's position at each request (as the
    entry reaches the logger: on the native backend right after the request's transaction)."""

    def __init__(self, sim: SimulatedEC | None = None) -> None:
        super().__init__()
        self.sim = sim
        self.tx_log: list[tuple[str, float, bytes, float | None, bool | None]] = []

    def __call__(self, direction: str, frame: bytes) -> None:
        super().__call__(direction, frame)
        if direction == "tx":
            pos = moving = None
            if self.sim is not None:
                pos, moving = self.sim.position_mm, self.sim.moving
            self.tx_log.append((threading.current_thread().name, time.perf_counter(), frame, pos, moving))

    def main_frames(self) -> list[bytes]:
        """Frames of the main thread (the binding runs a long call of the main thread on a helper thread named
        ``iad-call:MainThread``, so Ctrl+C stays possible)."""
        main = threading.main_thread().name
        return [f for name, _, f, _, _ in self.tx_log if name in (main, f"iad-call:{main}")]

    def clear(self) -> None:
        super().clear()
        self.tx_log.clear()


def open_sim(sim: SimulatedEC, *, min_gap_s: float = 0.0, **kwargs: object) -> tuple[Elecylinder, ThreadRecorder]:
    rec = ThreadRecorder(sim)
    kwargs.setdefault("poll_interval_s", 0.002)
    ec = Elecylinder(ModbusRTU(sim, frame_logger=rec, min_gap_s=min_gap_s), owns_bus=True, **kwargs)  # type: ignore[arg-type]
    return ec, rec


def lean_samples(rec: FrameRecorder) -> list[Status]:
    """The lean status samples (FC03 0x9000 x8) among the frames ``rec`` recorded, decoded from the replies: what a
    wait saw, also one the driver runs inside a single call."""
    seen: list[Status] = []
    lean = False
    for direction, frame in list(rec.frames):
        if direction == "tx":
            lean = is_status_read(frame, M.LEAN_STATUS_WORDS)
        elif direction == "rx" and lean:
            lean = False
            if frame[1] == 0x03:  # not an exception reply
                data = frame[3 : 3 + frame[2]]
                words = [int.from_bytes(data[i : i + 2], "big") for i in range(0, len(data), 2)]
                seen.append(Status.from_registers(words))
    return seen


def wait_until(predicate, timeout: float = 2.0) -> bool:  # type: ignore[no-untyped-def]
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        if predicate():
            return True
        time.sleep(0.002)
    return predicate()


# =========================================================================== the status monitor


class LeanStatusTest(unittest.TestCase):
    def test_one_read_of_eight_words(self) -> None:
        ec, sim, rec = make_ec()
        with ec:
            rec.clear()
            st = ec.read_lean_status()
            self.assertEqual(rec.tx, [bytes.fromhex("01 03 90 00 00 08".replace(" ", "")) + rec.tx[0][-2:]])
            self.assertEqual(st.position_mm, round(sim.position_mm, 2))
            self.assertTrue(st.servo_on and st.homed and st.in_position and st.modbus_enabled)
            self.assertIsNone(st.auto_servo_off)  # STAT is not in the lean block
            self.assertEqual(set(st.raw), {"PNOW", "ALMC", "DIPM", "DOPM", "DSS1", "DSS2", "DSSE"})


class MonitorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.ec, self.sim, self.rec = make_ec()

    def tearDown(self) -> None:
        self.ec.close()

    def test_publishes_lean_samples_and_stops_cleanly(self) -> None:
        mon = self.ec.start_monitor()
        self.assertIs(self.ec.monitor, mon)
        self.assertIs(self.ec.start_monitor(), mon, "a second start returns the running monitor")
        self.assertTrue(wait_until(lambda: mon.polls >= 20))
        first = mon.latest
        self.assertIsNotNone(first)
        self.assertTrue(wait_until(lambda: mon.latest.seq > first.seq))  # type: ignore[union-attr]
        sample = mon.latest
        assert sample is not None
        self.assertLessEqual(sample.t_start, sample.t)
        self.assertEqual(mon.timestamp, sample.t)
        self.assertIsInstance(mon.status, Status)
        self.assertIsNotNone(mon.fresh())
        reads = [f for f in self.rec.tx if is_status_read(f)]
        self.assertTrue(reads and all(is_status_read(f, M.LEAN_STATUS_WORDS) for f in reads), "monitor reads 8 words")
        self.assertTrue(mon.stop())
        self.assertFalse(mon.running)
        polls = mon.polls
        time.sleep(0.05)
        self.assertEqual(mon.polls, polls, "no poll after stop()")
        self.assertIsNone(mon.fresh(), "a stopped monitor has no fresh status")
        self.assertIsNone(mon.wait_update(timeout=0.01))

    def test_port_lock_is_free_between_polls(self) -> None:
        mon = self.ec.start_monitor()
        self.assertTrue(wait_until(lambda: mon.polls >= 5))
        worst = 0.0
        for _ in range(50):
            t0 = time.perf_counter()
            with self.ec.bus.exclusive():  # what another thread / axis needs to send
                worst = max(worst, time.perf_counter() - t0)
            time.sleep(0.001)
        self.assertLess(worst, 0.05)
        before = mon.polls
        self.assertTrue(wait_until(lambda: mon.polls > before + 5), "the monitor kept polling")

    def test_event_waiters_wake_on_the_matching_sample(self) -> None:
        mon = self.ec.start_monitor()
        self.ec.move_to(30.0, speed_mm_s=100.0, accel_g=0.3, wait=False)
        st = mon.wait_for(lambda s: s.position_mm < 50.0, timeout=5.0)
        self.assertIsNotNone(st)
        self.assertLess(st.position_mm, 50.0)  # type: ignore[union-attr]
        done = mon.wait_done(timeout=5.0)
        self.assertIsNotNone(done)
        self.assertTrue(fast_done(done, 30.0, 0.1))  # type: ignore[arg-type]
        self.assertIsNone(mon.wait_for(lambda s: False, timeout=0.05), "timeout gives None")
        with self.assertRaises(ZeroDivisionError):
            mon.wait_for(lambda s: 1 / 0 > 0, timeout=1.0)  # a predicate error reaches the waiting thread

    def test_on_change_and_on_done_callbacks(self) -> None:
        changes: list[tuple[Status, Status | None]] = []
        dones: list[Status] = []

        def broken(st: Status, previous: Status | None) -> None:
            raise RuntimeError("callback bug")

        self.ec.close()
        self.ec, self.sim, self.rec = make_ec(time_scale=5.0)  # slow enough for many samples during the move
        with self.assertLogs("iai_actuator_driver.monitor", "WARNING") as logs:
            mon = self.ec.start_monitor(on_change=[lambda st, prev: changes.append((st, prev)), broken])
            self.ec.start_monitor(on_done=dones.append)  # added to the running monitor
            self.assertTrue(wait_until(lambda: mon.polls >= 3))
            self.ec.move_to(40.0, speed_mm_s=100.0, accel_g=0.3, completion="fast")
            self.assertTrue(wait_until(lambda: len(dones) >= 1, 3.0))
        self.assertEqual(sum("callback" in line for line in logs.output), 1, "a failing callback is logged once")
        self.assertTrue(mon.running, "a failing callback must not stop the monitor")
        self.assertEqual(len(dones), 1)
        self.assertTrue(abs(dones[0].position_mm - 40.0) <= 0.1 + 1e-6, dones[0].position_mm)
        self.assertGreater(len(changes), 5)
        self.assertIsNone(changes[0][1])
        for st, prev in changes[1:]:
            self.assertIsNotNone(prev)
            self.assertNotEqual((st.raw, st.position_mm), (prev.raw, prev.position_mm))  # type: ignore[union-attr]

    def test_transient_errors_are_retried(self) -> None:
        mon = self.ec.start_monitor()
        self.assertTrue(wait_until(lambda: mon.polls >= 3))
        self.sim.inject_fault("timeout", 3)
        self.assertTrue(wait_until(lambda: mon.error_count >= 3))
        before = mon.polls
        self.assertTrue(wait_until(lambda: mon.polls > before + 3), "polling resumes after timeouts")
        self.assertTrue(mon.running)
        self.assertIsNone(mon.error)

    def test_port_vanishing_ends_the_monitor_and_wakes_waiters(self) -> None:
        mon = self.ec.start_monitor()
        self.assertTrue(wait_until(lambda: mon.polls >= 3))
        threading.Timer(0.05, self.sim.close).start()  # the USB adapter is unplugged
        t0 = time.perf_counter()
        with self.assertRaises(ModbusError):
            mon.wait_for(lambda s: False, timeout=5.0)
        self.assertLess(time.perf_counter() - t0, 2.0)
        self.assertTrue(wait_until(lambda: not mon.running))
        self.assertIsNotNone(mon.error)
        with self.assertRaises(ModbusError):
            mon.wait_update(timeout=0.01)
        self.assertFalse(self.ec.monitor is not None and self.ec.monitor.running, "waits fall back to their own reads")

    def test_failed_monitor_restarts_with_its_callbacks(self) -> None:
        seen: list[Status] = []
        self.ec.close()
        self.ec, self.sim, self.rec = make_ec(time_scale=5.0)  # slow enough to see the move start
        mon = self.ec.start_monitor(on_done=seen.append)
        self.assertTrue(wait_until(lambda: mon.polls >= 2))
        self.sim.unplug()  # the port vanishes: the monitor ends on the error
        self.assertTrue(wait_until(lambda: not mon.running))
        self.assertIsNotNone(mon.error)
        self.sim.replug()
        self.ec.bus.reopen()
        self.assertIs(self.ec.start_monitor(), mon)
        self.assertTrue(mon.running)
        self.assertIsNone(mon.error)
        self.ec.move_to(60.0, speed_mm_s=100.0, accel_g=0.3)
        self.assertTrue(wait_until(lambda: len(seen) >= 1))

    def test_suspend_pauses_polling(self) -> None:
        mon = self.ec.start_monitor()
        self.assertTrue(wait_until(lambda: mon.polls >= 3))
        with mon.suspend():
            polls = mon.polls
            time.sleep(0.08)
            self.assertEqual(mon.polls, polls)
            self.assertIsNone(mon.fresh(), "a suspended monitor goes stale")
        self.assertTrue(wait_until(lambda: mon.polls > polls))

    def test_close_stops_the_monitor(self) -> None:
        mon = self.ec.start_monitor()
        self.assertTrue(wait_until(lambda: mon.polls >= 1))
        self.ec.close()
        self.assertFalse(mon.running)
        self.assertIsNone(self.ec.monitor)
        self.assertIsNone(mon.error, "a clean stop is not an error")

    def test_bad_arguments(self) -> None:
        for kwargs in ({"gap_s": -1}, {"gap_s": "x"}, {"retries": -1}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                AxisMonitor(self.ec, **kwargs)  # type: ignore[arg-type]


class SharedPortMonitorTest(unittest.TestCase):
    def test_two_monitored_axes_on_one_port(self) -> None:
        line = sim_line([1, 2], name="L", time_scale=5)
        bus = ModbusRTU(line)
        axes = [Elecylinder(bus, slave=s, poll_interval_s=0.005, completion="fast") for s in (1, 2)]
        targets = {1: 20.0, 2: 130.0}
        results: dict[int, object] = {}
        monitors = [ax.start_monitor() for ax in axes]
        try:
            self.assertTrue(wait_until(lambda: all(m.polls >= 3 for m in monitors)))

            def drive(ax: Elecylinder) -> None:
                try:
                    results[ax.slave] = ax.move_to(targets[ax.slave], speed_mm_s=100, accel_g=0.3)
                except Exception as exc:  # reported below
                    results[ax.slave] = exc

            start = len(line.requests)
            threads = [threading.Thread(target=drive, args=(ax,)) for ax in axes]
            for t in threads:
                t.start()
            for t in threads:
                t.join(15)
            for s, target in targets.items():
                st = results[s]
                self.assertNotIsInstance(st, Exception, st)
                self.assertLessEqual(abs(st.position_mm - target), 0.1 + 1e-6)  # type: ignore[union-attr]
                self.assertLessEqual(abs(line[s].position_mm - target), 0.1 + 1e-6)
            slaves = [a for a in line.request_slaves()[start:] if a in (1, 2)]
            self.assertGreater(sum(1 for a, b in zip(slaves, slaves[1:]) if a != b), 10, "frames did not interleave")
            self.assertTrue(all(m.running and m.error is None for m in monitors))
        finally:
            for ax in axes:
                ax.stop_monitor()
            bus.close()


# =========================================================================== completion="fast"


class FastCompletionTest(unittest.TestCase):
    """The unit runs on its stepped clock (SteppedClockSim) and the driver on ``testing.virtual_clock``: the move's
    timeout, the settle hold and the settled-away rule count the polls' own sleeps, not the real time a busy PC takes
    for the polls (a wait fed by the monitor sleeps on nothing, so for it they never run out)."""

    def test_fast_returns_sooner_than_safe(self) -> None:
        counts = {}
        for mode in ("safe", "fast"):
            sim = SteppedClockSim(step_s=0.01, profile="gripper", position_mm=3.0)
            with virtual_clock():
                ec, rec = open_sim(sim)
                with ec:
                    ec.move_to(3.0, speed_mm_s=10.0)  # limits read, axis at rest
                    rec.clear()
                    st = ec.move_to(8.0, speed_mm_s=10.0, completion=mode)
                    self.assertLessEqual(abs(st.position_mm - 8.0), 0.1 + 1e-6)
                    after = rec.tx[next(i for i, f in enumerate(rec.tx) if is_move_write(f)) + 1:]
                    counts[mode] = sum(1 for f in after if is_status_read(f))
                    if mode == "fast":
                        self.assertTrue(all(is_status_read(f, M.LEAN_STATUS_WORDS) for f in after if is_status_read(f)))
                        self.assertTrue(st.in_position and not st.move_bit)
        self.assertLess(counts["fast"], counts["safe"])

    def test_fast_never_done_before_the_band_early_pend(self) -> None:
        """Gripper: PEND rises inside the band while the axis still moves (VERIFIED on the EC-GRBP8M); fast
        accepts the first sample that is inside the band with PEND and MOVE clear, never an earlier one."""
        sim = SteppedClockSim(step_s=0.01, profile="gripper", position_mm=3.0)
        with virtual_clock():
            ec, rec = open_sim(sim, completion="fast")
            with ec:
                ec.move_to(3.0, speed_mm_s=10.0)
                rec.clear()
                st = ec.move_to(8.0, speed_mm_s=10.0, band_mm=0.1)
                seen = lean_samples(rec)
        self.assertEqual(seen[-1], st)  # the result is the last sample read (decoded again from its frame)
        self.assertLessEqual(abs(st.position_mm - 8.0), 0.1 + 1e-6)
        for s in seen[:-1]:
            self.assertFalse(fast_done(s, 8.0, 0.1), s.position_mm)
        self.assertTrue(any(s.in_position for s in seen[:-1]) or st.position_mm < 8.0 or not sim.moving)

    def test_fast_never_done_before_the_band_stale_pend(self) -> None:
        """A stale PEND with MOVE clear right after the command (old position) never ends a fast wait; on the virtual
        clock the six stale polls cannot outlast the settled-away rule however slowly a busy PC sends them."""
        for use_monitor in (False, True):
            with self.subTest(monitor=use_monitor), virtual_clock():
                sim = SteppedClockSim(step_s=0.005, position_mm=20.0, stale_status_polls=6)
                ec, _ = open_sim(sim, completion="fast")
                with ec:
                    ec.move_to(20.0, speed_mm_s=100.0, accel_g=0.3)
                    if use_monitor:
                        ec.start_monitor()
                    for target in (60.0, 25.0, 25.5):
                        served = sim.stale_polls_served
                        st = ec.move_to(target, speed_mm_s=100.0, accel_g=0.3, band_mm=0.1)
                        self.assertLessEqual(abs(st.position_mm - target), 0.1 + 1e-6)
                        self.assertLessEqual(abs(sim.position_mm - target), 0.1 + 1e-6)
                        # the stale answers were read, counted over both pollers (the wait's own polls, or the
                        # monitor's that feed the wait): all six on the long moves; the 0.5 mm move reaches its band
                        # while they last, and a stale answer inside the band may end it
                        self.assertGreaterEqual(sim.stale_polls_served - served, 1 if target == 25.5 else 6, target)

    def test_per_axis_and_per_call(self) -> None:
        sim = SteppedClockSim(step_s=0.01, profile="gripper", position_mm=3.0)
        with virtual_clock():
            ec, rec = open_sim(sim, completion="fast")
            with ec:
                ec.move_to(3.0, speed_mm_s=10.0)
                rec.clear()
                st = ec.move_to(8.0, speed_mm_s=10.0, completion="safe")  # per call beats the axis
                self.assertEqual(st.position_mm, 8.0)
                self.assertFalse(sim.moving)
                self.assertTrue(any(is_status_read(f, R.STATUS_READ_WORDS) for f in rec.tx))
                ec.completion = "safe"
                rec.clear()
                ec.move_to(3.0, speed_mm_s=10.0, completion="fast")
                after = rec.tx[next(i for i, f in enumerate(rec.tx) if is_move_write(f)) + 1:]
                self.assertTrue(after and all(is_status_read(f, M.LEAN_STATUS_WORDS) for f in after
                                              if is_status_read(f)))  # fmt: skip
                st = ec.wait_idle(3.0, 0.1, timeout=5.0, completion="fast")
                self.assertTrue(fast_done(st, 3.0, 0.1))

    def test_bad_completion_sends_nothing(self) -> None:
        ec, _sim, rec = make_ec()
        with ec:
            with self.assertRaises(ValueError):
                Elecylinder(ec.bus, completion="quick")
            rec.clear()
            for call in (lambda: ec.move_to(80.0, completion="quick"),
                         lambda: ec.move_sequence([80.0], completion="quick"),
                         lambda: ec.wait_idle(completion="quick")):  # fmt: skip
                with self.assertRaises(ValueError):
                    call()
            self.assertEqual(rec.tx, [])

    def test_fast_profile_move(self) -> None:
        ec, _sim, _ = make_ec()
        with ec:
            st = ec.move_to(40.0, speed_mm_s=100.0, accel_g=0.3, decel_g=0.2, completion="fast")
            self.assertLessEqual(abs(st.position_mm - 40.0), 0.1 + 1e-6)

    def test_alarm_during_a_fast_wait_stops_and_raises(self) -> None:
        ec, sim, rec = make_ec(time_scale=2.0)
        with ec:
            ec.start_monitor()
            handle = ec.move_to(140.0, speed_mm_s=20.0, completion="fast", wait=False)
            time.sleep(0.05)
            sim.trigger_alarm(0x0C0)
            with self.assertRaises(EcAlarmError):
                handle.wait()
            self.assertTrue(any(is_stop_on(f) for f in rec.tx))


# =========================================================================== pre-move status from the monitor


class PreMoveStatusTest(unittest.TestCase):
    def test_fresh_monitor_status_makes_a_one_frame_move(self) -> None:
        sim = SimulatedEC(time_scale=50.0)
        ec, rec = open_sim(sim, min_gap_s=0.0005, completion="fast")
        with ec:
            ec.move_to(60.0, speed_mm_s=100.0, accel_g=0.3)  # limits read once
            mon = ec.start_monitor()
            self.assertTrue(wait_until(lambda: mon.fresh() is not None))
            rec.clear()
            handle = ec.move_to(80.0, speed_mm_s=100.0, accel_g=0.3, wait=False)
            main = rec.main_frames()
            self.assertEqual(len(main), 1, [f.hex(" ") for f in main])
            self.assertTrue(is_move_write(main[0]))
            st = handle.wait()
            self.assertLessEqual(abs(st.position_mm - 80.0), 0.1 + 1e-6)
            self.assertFalse(any(is_status_read(f) for f in rec.main_frames()), "the wait used the monitor's samples")

    def test_stale_monitor_status_is_read_again(self) -> None:
        sim = SimulatedEC(time_scale=50.0)
        ec, rec = open_sim(sim)
        with ec:
            ec.move_to(60.0, speed_mm_s=100.0, accel_g=0.3)
            mon = ec.start_monitor()
            self.assertTrue(wait_until(lambda: mon.polls >= 2))
            with mon.suspend():
                time.sleep(0.06)  # older than FRESH_S
                rec.clear()
                handle = ec.move_to(70.0, speed_mm_s=100.0, accel_g=0.3, wait=False)
                main = rec.main_frames()
                self.assertTrue(is_status_read(main[0], R.STATUS_READ_WORDS))
                self.assertTrue(is_move_write(main[-1]))
            handle.wait()

    def test_monitor_status_not_ready_is_read_again(self) -> None:
        sim = SimulatedEC(time_scale=50.0)
        ec, _ = open_sim(sim)
        with ec:
            ec.move_to(60.0, speed_mm_s=100.0, accel_g=0.3)
            mon = ec.start_monitor()
            sim.servo_on = False
            self.assertTrue(wait_until(lambda: mon.status is not None and not mon.status.servo_on))
            with self.assertRaises(Exception) as caught:
                ec.move_to(70.0, speed_mm_s=100.0, accel_g=0.3)
            self.assertIn("servo", str(caught.exception))
            self.assertFalse(sim.moving)


# =========================================================================== MoveHandle


class MoveHandleTest(unittest.TestCase):
    def setUp(self) -> None:
        self.ec, self.sim, self.rec = make_ec(time_scale=20.0)

    def tearDown(self) -> None:
        self.ec.close()

    def test_wait_done_result(self) -> None:
        handle = self.ec.move_to(120.0, speed_mm_s=100.0, accel_g=0.3, wait=False)
        self.assertIsInstance(handle, MoveHandle)
        self.assertFalse(handle.done())
        self.assertIsNone(handle.result)
        st = handle.wait()
        self.assertIs(handle.result, st)
        self.assertTrue(handle.done())
        self.assertIs(handle.wait(), st)
        self.assertAlmostEqual(st.position_mm, 120.0, delta=0.1)
        data = json.loads(json.dumps(handle.to_dict()))
        self.assertEqual((data["target_mm"], data["done"], data["cancelled"]), (120.0, True, False))
        self.assertIn("done", repr(handle))

    def test_done_polling_fast_with_monitor(self) -> None:
        self.ec.start_monitor()
        handle = self.ec.move_to(30.0, speed_mm_s=100.0, accel_g=0.3, wait=False, completion="fast")
        self.assertTrue(wait_until(handle.done, 10.0))
        self.assertTrue(fast_done(handle.result, 30.0, 0.1))  # type: ignore[arg-type]

    def test_done_polling_safe(self) -> None:
        handle = self.ec.move_to(30.0, speed_mm_s=100.0, accel_g=0.3, wait=False)
        self.assertTrue(wait_until(handle.done, 10.0))
        self.assertAlmostEqual(handle.result.position_mm, 30.0, delta=0.1)  # type: ignore[union-attr]
        self.assertFalse(self.sim.moving)

    def test_cancel_is_stop(self) -> None:
        self.ec.start_monitor()
        handle = self.ec.move_to(140.0, speed_mm_s=20.0, wait=False)
        time.sleep(0.05)
        self.rec.clear()
        st = handle.cancel()
        own = [f for f in self.rec.tx if not is_status_read(f, M.LEAN_STATUS_WORDS)]  # without the monitor's polls
        self.assertTrue(is_stop_on(own[0]), "STOP is the first frame of cancel()")
        self.assertFalse(self.sim.moving)
        self.assertTrue(handle.cancelled)
        self.assertIs(handle.result, st)
        self.assertIs(handle.wait(), st)
        self.assertLess(st.position_mm, 139.0)  # type: ignore[union-attr]

    def test_wait_timeout_stops(self) -> None:
        handle = self.ec.move_to(140.0, speed_mm_s=20.0, wait=False)
        with self.assertRaises(MotionTimeoutError):
            handle.wait(timeout=0.05)
        self.assertFalse(self.sim.moving)

    def test_relative_moves_return_handles_and_checks_still_run(self) -> None:
        self.assertIsInstance(self.ec.move_by(5.0, speed_mm_s=50.0, wait=False), MoveHandle)
        handle = self.ec.jog(-1.0, wait=False, completion="fast")
        self.assertIsInstance(handle, MoveHandle)
        self.assertEqual(handle.completion, "fast")
        handle.wait()
        self.rec.clear()
        with self.assertRaises(SoftLimitError):
            self.ec.move_to(150.5, wait=False)
        with self.assertRaises(SoftLimitError):
            self.ec.move_to(80.0, speed_mm_s=500.0, wait=False)
        self.assertFalse(any(is_move_write(f) for f in self.rec.tx))


# =========================================================================== blended sequences


class MoveSequenceTest(unittest.TestCase):
    def setUp(self) -> None:
        # a stepped clock: the axis moves at most 2 mm per request (100 mm/s), so a poll falls inside the 2 mm blend
        # band and the axis still moves when the next target goes out, whatever the PC's load (at time_scale 5 the
        # band is crossed in 4 ms of real time, less than a scheduling delay)
        self.sim = SteppedClockSim(step_s=0.02)
        self.ec, self.rec = open_sim(self.sim, poll_interval_s=0.005)
        self.ec.move_to(40.0, speed_mm_s=100.0, accel_g=0.3)

    def tearDown(self) -> None:
        self.ec.close()

    def move_writes(self) -> list[tuple[bytes, float | None, bool | None]]:
        """The 0x9900 writes, each with the unit's position and motion when it reached the unit: those of the last
        status the unit answered before it. The stepped clock moves only on a request, and this thread's frames come in
        wire order (a frame logger entry may come after its transaction, so the simulator is not asked at that time)."""
        writes: list[tuple[bytes, float | None, bool | None]] = []
        last: tuple[float | None, bool | None] = (None, None)
        status_read = False
        for direction, frame in list(self.rec.frames):
            if direction == "tx":
                status_read = is_status_read(frame)
                if is_move_write(frame):
                    writes.append((frame, *last))
            elif direction == "rx" and status_read:
                status_read = False
                if frame[1] == 0x03:  # not an exception reply
                    data = frame[3 : 3 + frame[2]]
                    st = Status.from_registers([int.from_bytes(data[i : i + 2], "big") for i in range(0, len(data), 2)])
                    last = (st.position_mm, st.moving)
        return writes

    def test_without_blend_each_point_completes(self) -> None:
        self.rec.clear()
        results = self.ec.move_sequence([50.0, 60.0, 55.0], speed_mm_s=100.0, accel_g=0.3)
        self.assertEqual(len(results), 3)
        for st, target in zip(results, (50.0, 60.0, 55.0)):
            self.assertAlmostEqual(st.position_mm, target, delta=0.1)
        writes = self.move_writes()
        self.assertEqual(len(writes), 3)
        for _, pos, moving in writes[1:]:
            self.assertFalse(moving, "the next point went out only after the previous one completed")

    def test_blend_sends_the_next_target_inside_the_band(self) -> None:
        self.rec.clear()
        points = [60.0, {"mm": 90.0, "speed_mm_s": 80.0}, 70.0]
        results = self.ec.move_sequence(points, speed_mm_s=100.0, accel_g=0.3, blend_band=2.0, completion="fast")
        self.assertEqual(len(results), 3)
        self.assertLessEqual(abs(results[0].position_mm - 60.0), 2.0 + 1e-6)
        self.assertLessEqual(abs(results[1].position_mm - 90.0), 2.0 + 1e-6)
        self.assertTrue(fast_done(results[2], 70.0, 0.1))
        writes = self.move_writes()
        self.assertEqual(len(writes), 3)
        for (_, pos, moving), previous in zip(writes[1:], (60.0, 90.0)):
            self.assertTrue(moving, "a blended point must not stop the axis")
            self.assertLessEqual(abs(pos - previous), 2.5, (pos, previous))  # type: ignore[operator]
        self.assertEqual(int.from_bytes(writes[1][0][15:19], "big"), 8000)  # VCMD 80.00 mm/s for point 1

    def test_every_point_is_checked_before_anything_is_sent(self) -> None:
        self.rec.clear()
        with self.assertRaises(SoftLimitError) as caught:
            self.ec.move_sequence([50.0, 60.0, 151.0], blend_band=1.0)
        self.assertIn("point 2", " ".join(getattr(caught.exception, "__notes__", [])))
        with self.assertRaises(SoftLimitError):
            self.ec.move_sequence([50.0, {"mm": 60.0, "speed_mm_s": 900.0}])
        for band in (0.0, 20.0, math.nan, True):
            with self.subTest(band=band), self.assertRaises(SoftLimitError):
                self.ec.move_sequence([50.0, 60.0], blend_band=band)  # type: ignore[arg-type]
        for points in ([], [{"to": 50.0}], [{"mm": 50.0, "decel_g": 0.1}]):
            with self.subTest(points=points), self.assertRaises(ValueError):
                self.ec.move_sequence(points)  # type: ignore[arg-type]
        self.assertEqual(self.rec.tx, [])
        self.assertFalse(self.sim.moving)


# =========================================================================== Axis.open(fast=True) and STOP


class FastOpenTest(unittest.TestCase):
    def test_open_fast_raises_baud_and_turns_everything_on(self) -> None:
        sim = SimulatedEC(baudrate=38400, time_scale=50.0)
        ax = Axis.open(sim, fast=True, poll_interval_s=0.005)
        try:
            self.assertEqual(ax.baudrate, 230400)
            self.assertEqual(sim.ec_baudrate, 230400)
            self.assertEqual(ax.completion, "fast")
            self.assertTrue(ax.fast_mode)
            self.assertEqual(ax.fast_notes, [])
            mon = ax.monitor
            self.assertIsNotNone(mon)
            self.assertTrue(mon.running)  # type: ignore[union-attr]
            ax.connect(fast=True)  # idempotent
            self.assertIs(ax.monitor, mon)
            st = ax.move_to(100.0, speed_mm_s=100.0, accel_g=0.3)
            self.assertTrue(fast_done(st, 100.0, 0.1))
        finally:
            ax.close()
        self.assertFalse(mon.running)  # type: ignore[union-attr]

    def test_connect_fast_without_a_baud_rate(self) -> None:
        ec, _sim, _ = make_ec()
        with ec:
            ec.connect(fast=True)
            self.assertEqual(ec.completion, "fast")
            self.assertTrue(ec.monitor is not None and ec.monitor.running)
            self.assertTrue(any("no baud rate" in n for n in ec.fast_notes), ec.fast_notes)
            self.assertIsNone(ec.baudrate)

    def test_shared_port_keeps_the_rate(self) -> None:
        sim = SimulatedEC(baudrate=38400, time_scale=50.0)
        bus = ModbusRTU(sim)
        ax = Elecylinder(bus, poll_interval_s=0.005)
        try:
            summary = ax.enable_fast(monitor=False)
            self.assertEqual(summary["baudrate"], 38400)
            self.assertFalse(summary["monitor"])
            self.assertTrue(any("shared port" in n for n in summary["notes"]))
            self.assertEqual(ax.completion, "fast")
        finally:
            bus.close()

    def test_stop_stays_immediate_with_the_monitor(self) -> None:
        sim = SimulatedEC(time_scale=1.0)
        ec, rec = open_sim(sim, completion="fast")
        with ec:
            mon = ec.start_monitor(gap_s=0.0)  # worst case: polls back to back
            ec.move_to(140.0, speed_mm_s=20.0, accel_g=0.3, wait=False)
            self.assertTrue(wait_until(lambda: mon.polls >= 20))
            t0 = time.perf_counter()
            ec.stop()
            stop_at = next(t for _, t, f, _, _ in rec.tx_log if t >= t0 and is_stop_on(f))
            self.assertLess(stop_at - t0, 0.05)
            self.assertFalse(sim.moving)
            self.assertTrue(mon.running)


if __name__ == "__main__":
    unittest.main()
