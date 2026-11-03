"""Fixes of the round-5 review (the "polish" commit): calls on one axis from several threads keep to their own results
and their own events, and a Ctrl+C note says what a read did.

Simulator only: no COM port is opened. The tests that name a call of the core (``_after``) check that each call returns its own
result.
"""

from __future__ import annotations

import _thread
import threading
import time
import unittest
from unittest import mock

from iai_actuator_driver import info
from iai_actuator_driver import recovery as rc
from iai_actuator_driver.ec import Elecylinder
from iai_actuator_driver.sim import SimulatedEC

from ec_testutil import FrameRecorder

FAST = {"poll_interval_s": 0.005}


def open_axis(sim: SimulatedEC, **kwargs: object) -> Elecylinder:
    ax = Elecylinder.open(sim, frame_logger=FrameRecorder(), **{**FAST, **kwargs})  # type: ignore[arg-type]
    ax.connect()
    return ax


class Events:
    def __init__(self) -> None:
        self.items: list[rc.RecoveryEvent] = []

    def __call__(self, event: rc.RecoveryEvent) -> None:
        self.items.append(event)

    @property
    def kinds(self) -> list[str]:
        return [e.kind for e in self.items]


def after(function: str, then: object) -> object:
    """A call on another thread (``then``) replaces the handle's last result just as ``function`` returns."""
    from iai_actuator_driver import native

    real = native._invoke

    def invoke(lib: object, name: str, *args: object) -> object:
        outcome = real(lib, name, *args)  # type: ignore[arg-type]
        if name == function:
            other = threading.Thread(target=then)  # type: ignore[arg-type]
            other.start()
            other.join(5.0)
        return outcome

    return mock.patch.object(native, "_invoke", invoke)


class AxisCallsKeepTheirOwnResultTest(unittest.TestCase):
    """A link check on another thread replaces the handle's result as the call under test returns (it does so at that
    very moment in each test): the call gives its own result."""

    def test_write_parameter_number(self) -> None:
        ax = open_axis(SimulatedEC(profile="gripper", time_scale=50))
        with after("iad_axis_write_parameter_number", lambda: rc.check_link(ax)):
            for _ in range(3):
                result = ax.write_parameter(5, 0, commit=False)
                self.assertEqual(result.no, 5)
        ax.close()

    def test_write_parameter_text(self) -> None:
        ax = open_axis(SimulatedEC(profile="gripper", time_scale=50))
        with after("iad_axis_write_parameter", lambda: rc.check_link(ax)):
            result = ax.write_parameter(5, "0", commit=False)
            self.assertEqual(result.no, 5)
        ax.close()

    def test_software_reset(self) -> None:
        ax = open_axis(SimulatedEC(profile="rod", time_scale=50))
        with after("iad_axis_software_reset", lambda: rc.check_link(ax)):
            result = ax.software_reset(reset_wait_s=0.0)
            self.assertTrue(result.accepted)
            self.assertTrue(result.notes is not None)
        ax.close()

    def test_set_baudrate(self) -> None:
        ax = open_axis(SimulatedEC(profile="rod", time_scale=50, baudrate=115200))
        with after("iad_axis_set_baud_rate", lambda: rc.check_link(ax)):
            result = ax.set_baudrate(230400, settle_s=0.0)
            self.assertEqual((result.previous, result.baudrate), (115200, 230400))
        ax.close()

    def test_push(self) -> None:
        ax = open_axis(SimulatedEC(profile="gripper", time_scale=50))
        ax.move_to(3.0, speed_mm_s=20.0)
        with after("iad_axis_push", lambda: rc.check_link(ax)):
            result = ax.push(4.0, 5.0, 60.0, speed_mm_s=20.0)
            self.assertTrue(result.missed)
        ax.close()

    def test_unit_info_longer_than_the_first_buffer(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=50)
        ax = open_axis(sim)
        with after("iad_axis_info_call_json", lambda: rc.check_link(ax)):
            for _ in range(3):
                data = info.unit_info(ax)
                self.assertIn("identity", data)
                self.assertNotIn("state", data)  # a link check's record has "state" and "ok"
        ax.close()

    def test_backup_longer_than_the_first_buffer(self) -> None:
        ax = open_axis(SimulatedEC(profile="gripper", time_scale=50))
        with after("iad_axis_info_call_json", lambda: rc.check_link(ax)):
            data = info.backup(ax)
            self.assertNotIn("state", data)
            self.assertTrue(data.get("parameters") or data.get("rows") or data.get("identity"), list(data))
        ax.close()


class LinkEventsOfOverlappingCallsTest(unittest.TestCase):
    def test_a_link_event_goes_to_one_of_two_overlapping_checks_not_to_both(self) -> None:
        for _ in range(8):
            sim = SimulatedEC(profile="gripper", time_scale=50)
            ax = open_axis(sim)
            rc.track(ax, name="G")
            first, second = Events(), Events()
            barrier = threading.Barrier(2)
            sim.unplug()

            def check(events: Events) -> None:
                barrier.wait(5.0)
                rc.check_link(ax, on_event=events)

            threads = [threading.Thread(target=check, args=(e,)) for e in (first, second)]
            for t in threads:
                t.start()
            for t in threads:
                t.join(10.0)
            # one link_lost happened: one call saw it happen, the other found the link already lost
            self.assertEqual(first.kinds + second.kinds, ["link_lost"], (first.kinds, second.kinds))
            ax.close()


class MonitorStartedWhileACallEndsTest(unittest.TestCase):
    def test_the_event_thread_a_call_leaves_is_kept_for_a_monitor_that_is_starting(self) -> None:
        from iai_actuator_driver import monitor as nm

        class Gated(SimulatedEC):
            gate: threading.Event | None = None
            entered = threading.Event()

            def on_request(self, frame: bytes) -> bytes | None:
                gate = self.gate
                if gate is not None and not self.entered.is_set():
                    self.entered.set()  # the first request of the call under test waits here
                    gate.wait(10.0)
                return NotImplemented  # type: ignore[return-value]

        sim = Gated(profile="gripper", time_scale=50)
        ax = open_axis(sim)
        rc.track(ax, name="G")
        sim.gate = threading.Event()
        call_events = Events()
        call = threading.Thread(target=lambda: rc.check_link(ax, on_event=call_events))
        call.start()
        self.assertTrue(sim.entered.wait(5.0))  # the call holds the axis's event thread (it listens)

        in_options = threading.Event()
        window = threading.Event()
        real_options = nm.AxisMonitor._options

        def slow_options(self: object) -> object:  # widens the time between taking the event thread and the start
            in_options.set()
            window.wait(10.0)
            return real_options(self)  # type: ignore[arg-type]

        changes: list[object] = []
        monitor: list[object] = []
        with mock.patch.object(nm.AxisMonitor, "_options", slow_options):
            starter = threading.Thread(target=lambda: monitor.append(ax.start_monitor(on_change=lambda s, p: changes.append(s))))
            starter.start()
            self.assertTrue(in_options.wait(5.0))  # the monitor has the event thread, its core start is yet to come
            sim.gate.set()  # the call ends now and lets go of the event thread
            call.join(10.0)
            window.set()
            starter.join(10.0)
        self.assertTrue(monitor and monitor[0].running)  # type: ignore[attr-defined]
        ax.servo(False)
        ax.servo(True)
        deadline = time.perf_counter() + 5.0
        while len(changes) < 2 and time.perf_counter() < deadline:
            time.sleep(0.01)
        self.assertGreaterEqual(len(changes), 2)  # the monitor's on_change still has its event thread
        ax.close()


class _Slow(SimulatedEC):
    """A unit that takes its time to answer, so that a long read is still running when Ctrl+C arrives."""

    def on_request(self, frame: bytes) -> bytes | None:
        time.sleep(0.004)
        return NotImplemented  # type: ignore[return-value]


class CtrlCNoteOfAReadTest(unittest.TestCase):
    def test_a_cancelled_unit_info_says_it_was_a_read_not_a_stop(self) -> None:
        ax = open_axis(_Slow(profile="gripper", time_scale=50))
        timer = threading.Timer(0.3, _thread.interrupt_main)
        timer.start()
        try:
            with self.assertRaises(KeyboardInterrupt) as raised:
                info.unit_info(ax)
        finally:
            timer.cancel()
        notes = " ".join(getattr(raised.exception, "__notes__", []))
        self.assertIn("read cancelled on Ctrl+C", notes)
        self.assertNotIn("decel stop", notes)
        ax.close()


if __name__ == "__main__":
    unittest.main()
