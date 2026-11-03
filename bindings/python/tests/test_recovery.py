"""Power-loss / link-loss recovery (iai_actuator_driver.recovery, Cell.on_fault / recover_all) and the simulator's faults.

Simulator only: no serial port is opened.
"""

from __future__ import annotations

import logging
import struct
import threading
import time
import unittest
from unittest import mock

from iai_actuator_driver import recovery as rc
from iai_actuator_driver import registers as R
from iai_actuator_driver.cell import Cell, CellError
from iai_actuator_driver.ec import Elecylinder
from iai_actuator_driver.errors import (
    EcAlarmError,
    ModbusError,
    ModbusException,
    ModbusTimeout,
    MotionTimeoutError,
    SoftLimitError,
)
from iai_actuator_driver.check import FAIL, SKIP, run_check
from iai_actuator_driver.info import backup
from iai_actuator_driver.modbus import ModbusRTU, TransportError
from iai_actuator_driver.sim import SimException, SimulatedEC
from iai_actuator_driver.sim_line import SimLine, sim_ports, sim_rcon_line

from ec_testutil import FrameRecorder, ScriptedTransport

FAST = {"poll_interval_s": 0.005}
OPEN = {**FAST, "scan_timeout": 0.01}
QUICK = rc.RecoveryPolicy(poll_s=0.02)
logging.getLogger("iai_actuator_driver.recovery").setLevel(logging.ERROR)  # every event is also logged as a warning

# Frames that would move the axis: the 0x9900 / 0x0F00 blocks, a 0x9800 row number, the HOME / SON / CSTR / PCRF coils,
# and the IAI motion / servo / home commands.
_MOTION_COILS = {R.COIL_HOME, R.COIL_SON, 0x040C, 0x042E}
_MOTION_REGISTERS = {0x9900, 0x9800, 0x0F00, 0x0D00, 0x9801}


def motion_frames(frames: list[bytes]) -> list[str]:
    """Every frame in ``frames`` that could start motion or switch the servo on (hex)."""
    bad = []
    for f in frames:
        if len(f) < 4:
            continue
        fc = f[1]
        if fc == 0x05 and struct.unpack(">H", f[2:4])[0] in _MOTION_COILS and f[4] == 0xFF:
            bad.append(f.hex(" "))
        elif fc in (0x06, 0x10, 0x17) and struct.unpack(">H", f[2:4])[0] in _MOTION_REGISTERS:
            bad.append(f.hex(" "))
        elif fc == 0x45 and f[0] != 0 and struct.unpack(">H", f[2:4])[0] in (0x06, 0x07, 0x09, 0x0A):
            bad.append(f.hex(" "))
    return bad


def open_axis(sim: SimulatedEC | SimLine, **kwargs: object) -> tuple[Elecylinder, FrameRecorder]:
    rec = FrameRecorder()
    ax = Elecylinder.open(sim, frame_logger=rec, **{**FAST, **kwargs})  # type: ignore[arg-type]
    ax.connect()
    return ax, rec


class Events:
    def __init__(self) -> None:
        self.items: list[rc.RecoveryEvent] = []

    def __call__(self, event: rc.RecoveryEvent) -> None:
        self.items.append(event)

    @property
    def kinds(self) -> list[str]:
        return [e.kind for e in self.items]


# ====================================================================== simulator fault injection


class SimFaultInjectionTest(unittest.TestCase):
    def test_power_cycle_resets_baud_pmss_uptime_home_and_logs_power_up(self) -> None:
        sim = SimulatedEC(profile="rod", baudrate=115200, time_scale=50)
        bus = ModbusRTU(sim, min_gap_s=0.0)
        time.sleep(0.05)
        uptime_before = R.from_u32_words(*bus.read_registers(1, R.STIM, 2))
        sim.power_cycle()
        self.assertEqual((sim.ec_baudrate, sim.pmsl, sim.homed, sim.safety_speed), (38400, False, False, False))
        bus.set_baudrate(38400)
        self.assertLess(R.from_u32_words(*bus.read_registers(1, R.STIM, 2)), uptime_before)
        self.assertEqual(bus.read_registers(1, R.ALARM_HISTORY_BASE, 6)[3], R.ALARM_POWER_UP)
        self.assertIn("HEND lost at power-up", sim.events)

    def test_absolute_units_keep_home_and_lose_home_can_be_forced_either_way(self) -> None:
        sim = SimulatedEC(profile="gripper")
        sim.power_cycle()
        self.assertTrue(sim.homed)  # WA absolute
        sim.power_cycle(lose_home=True)
        self.assertFalse(sim.homed)
        rod = SimulatedEC(profile="rod")
        rod.power_cycle(lose_home=False)  # one power-up of the EC-R6S rod still read HEND 1
        self.assertTrue(rod.homed)

    def test_power_off_is_silent_until_power_on(self) -> None:
        sim = SimulatedEC(time_scale=50)
        bus = ModbusRTU(sim, min_gap_s=0.0, retries=0)
        bus.write_coil(1, R.COIL_PMSL, True)
        sim.power_off()
        with self.assertRaises(ModbusTimeout):
            bus.read_registers(1, R.PNOW, 2)
        sim.power_on()
        self.assertEqual(bus.read_registers(1, R.DSSE, 1)[0] & R.DSSE_PMSS, 0)  # PMSS 0 after power-up

    def test_power_off_stops_a_move_where_it_is(self) -> None:
        sim = SimulatedEC(time_scale=5)
        ax, _ = open_axis(sim)
        ax.move_to(140.0, speed_mm_s=100, wait=False)
        time.sleep(0.05)
        sim.power_off()
        held = sim.position_mm
        time.sleep(0.05)
        self.assertEqual(sim.position_mm, held)
        self.assertFalse(sim.moving)
        sim.power_on()
        self.assertEqual(sim.position_mm, held)  # absolute slider: position kept

    def test_unplug_kills_the_handle_until_replug_and_reopen(self) -> None:
        sim = SimulatedEC(time_scale=50)
        bus = ModbusRTU(sim, min_gap_s=0.0)
        sim.unplug()
        self.assertFalse(sim.present)
        with self.assertRaises(TransportError) as caught:
            bus.read_registers(1, R.PNOW, 2)
        self.assertIsInstance(caught.exception.__cause__, OSError)
        self.assertIsInstance(caught.exception, ModbusError)
        with self.assertRaises(OSError):
            sim.reopen()  # still missing
        sim.replug()
        with self.assertRaises(TransportError):
            bus.read_registers(1, R.PNOW, 2)  # the old handle stays dead
        bus.reopen()
        self.assertEqual(len(bus.read_registers(1, R.PNOW, 2)), 2)
        self.assertEqual(sim.reopens, 1)
        self.assertTrue(sim.pmsl)  # the unit kept its power and state

    def test_simline_unplug_replug_reopen_and_power_cycle(self) -> None:
        line = SimLine.with_slaves([1, 2], name="L", time_scale=50)
        bus = ModbusRTU(line, min_gap_s=0.0)
        line.unplug()
        with self.assertRaises(TransportError):
            bus.read_registers(2, R.PNOW, 2)
        with self.assertRaises(TransportError):
            bus.reopen()
        line.replug()
        bus.reopen()
        self.assertEqual(len(bus.read_registers(2, R.PNOW, 2)), 2)
        line.power_cycle([2])
        self.assertTrue(line[1].pmsl)
        self.assertFalse(line[2].pmsl)
        bus.close()
        self.assertTrue(line.closed)
        bus.reopen()  # a closed line opens again, its units too
        self.assertEqual(len(bus.read_registers(1, R.PNOW, 2)), 2)

    def test_gateway_line_unplug(self) -> None:
        line = sim_rcon_line([1], time_scale=50)
        bus = ModbusRTU(line, min_gap_s=0.0)
        line.unplug()
        with self.assertRaises(TransportError):
            bus.read_registers(1, R.PNOW, 2)
        line.replug()
        bus.reopen()
        self.assertEqual(len(bus.read_registers(1, R.PNOW, 2)), 2)

    def test_serial_number_override(self) -> None:
        sim = SimulatedEC(profile="gripper", serial_number="AB-123")
        self.assertEqual(sim.serial_number, "AB-123")
        ax, _ = open_axis(sim)
        info = ax.identify()
        self.assertEqual(info.serial_number, "AB-123")
        self.assertEqual(info.model_name, "EC-GRBP8M-20-1-1-PN-WA-WL2")  # the rest of the block is unchanged

    def test_pcon_raises_0x069_again_at_power_up(self) -> None:
        sim = SimulatedEC(profile="pcon")
        bus = ModbusRTU(sim, min_gap_s=0.0)
        bus.write_coil(1, R.COIL_ALRS, True)  # the reset pulse: the alarm comes straight back while the clock is unset
        bus.write_coil(1, R.COIL_ALRS, False)
        self.assertEqual(sim.alarm_code, R.ALARM_RTC_STOP)
        sim.power_cycle()
        self.assertEqual(sim.alarm_code, R.ALARM_RTC_STOP)


# ====================================================================== bus reopen


class ModbusReopenTest(unittest.TestCase):
    def test_reopen_needs_a_reopenable_transport(self) -> None:
        bus = ModbusRTU(ScriptedTransport())
        self.assertFalse(bus.can_reopen)
        with self.assertRaises(TypeError):
            bus.reopen()

    def test_a_dead_handle_is_a_transport_error_and_not_retried(self) -> None:
        sim = SimulatedEC()
        bus = ModbusRTU(sim, min_gap_s=0.0, retries=3)
        sim.unplug()
        before = len(sim.requests)
        with self.assertRaises(TransportError):
            bus.write_coil(1, R.COIL_STOP, True)
        self.assertEqual(len(sim.requests), before)


# ====================================================================== detection


class _NamedPort:
    """A transport that only has a serial-port name (like SerialTransport), for port_present()."""

    def __init__(self, port: str) -> None:
        self.port = port

    def write(self, data: bytes) -> None:
        raise OSError("gone")

    def read(self, n: int) -> bytes:
        return b""

    def reset_input_buffer(self) -> None:
        pass

    def close(self) -> None:
        pass


class ClassifyTest(unittest.TestCase):
    def test_kinds(self) -> None:
        dead = TransportError("transport error: gone")
        dead.__cause__ = OSError("gone")
        wrapped = RuntimeError("move failed")
        wrapped.__cause__ = dead
        cases = [
            (dead, rc.PORT_VANISHED),
            (OSError("could not open port"), rc.PORT_VANISHED),
            (wrapped, rc.PORT_VANISHED),
            (ModbusTimeout("no answer"), rc.UNIT_SILENT),
            (ModbusException(2, 3), None),
            (SoftLimitError("too far"), None),
            (EcAlarmError(0x0C0), None),
            (MotionTimeoutError("slow"), None),
            (ModbusError("bus is closed"), None),
            (rc.UnitReplacedError("x", expected={}, found={}), rc.UNIT_REPLACED),
        ]
        for exc, kind in cases:
            with self.subTest(exc=type(exc).__name__):
                self.assertEqual(rc.classify_failure(exc), kind)

    def test_a_silent_unit_on_a_port_no_longer_listed_is_a_vanished_port(self) -> None:
        ax = Elecylinder(ModbusRTU(_NamedPort("COM7")))
        listed = [{"port": "COM3"}, {"port": "com7"}]
        self.assertTrue(rc.port_present(ax, lister=lambda: listed))
        self.assertEqual(rc.classify_failure(ModbusTimeout("x"), ax, lister=lambda: listed), rc.UNIT_SILENT)
        self.assertFalse(rc.port_present(ax, lister=lambda: listed[:1]))
        self.assertEqual(rc.classify_failure(ModbusTimeout("x"), ax, lister=lambda: listed[:1]), rc.PORT_VANISHED)
        self.assertIsNone(rc.port_present(Elecylinder(ModbusRTU(ScriptedTransport()))))
        self.assertEqual(rc.port_name(ax), "COM7")


class CheckLinkTest(unittest.TestCase):
    def setUp(self) -> None:
        self.sim = SimulatedEC(profile="gripper", time_scale=50)
        self.ax, self.rec = open_axis(self.sim)
        time.sleep(0.02)
        self.session = rc.track(self.ax, name="G")
        self.events = Events()

    def check(self) -> rc.LinkCheck:
        return rc.check_link(self.ax, on_event=self.events)

    def test_healthy_link_moves_the_baseline_forward(self) -> None:
        first = self.session.uptime_ms
        time.sleep(0.02)
        chk = self.check()
        self.assertTrue(chk.ok)
        self.assertEqual(chk.reasons, ())
        self.assertGreater(self.session.uptime_ms, first)
        self.assertEqual(self.events.items, [])
        reads = [f for f in self.rec.tx[-1:]]
        self.assertEqual(reads[0][1], 0x03)  # one status read, nothing written

    def test_power_cycle_is_a_restart(self) -> None:
        self.sim.power_cycle()
        chk = self.check()
        self.assertEqual(chk.state, rc.UNIT_RESTARTED)
        self.assertTrue(any("uptime counter dropped" in r for r in chk.reasons), chk.reasons)
        self.assertTrue(any("PMSS" in r for r in chk.reasons), chk.reasons)
        self.check()
        self.assertEqual(self.events.kinds, ["unit_restarted"])  # once per change
        self.assertEqual(rc.link_state(self.ax), rc.UNIT_RESTARTED)

    def test_power_off_is_a_silent_unit_and_unplug_a_vanished_port(self) -> None:
        self.sim.power_off()
        chk = self.check()
        self.assertEqual(chk.state, rc.UNIT_SILENT)
        self.assertIsInstance(chk.error, ModbusTimeout)
        self.check()
        self.assertEqual(self.events.kinds, ["link_lost"])
        self.sim.power_on()
        self.sim.unplug()
        chk = self.check()
        self.assertEqual(chk.state, rc.PORT_VANISHED)
        self.assertEqual(self.events.kinds, ["link_lost"])  # still lost: no new event

    def test_uptime_behind_the_pc_clock_is_a_restart(self) -> None:
        self.session.uptime_at = time.perf_counter() - 3600.0  # the PC was away for an hour; the unit shows less
        chk = self.check()
        self.assertEqual(chk.state, rc.UNIT_RESTARTED)
        self.assertIn("behind", chk.reasons[0])

    def test_a_deliberate_control_change_noted_is_not_a_restart(self) -> None:
        self.ax.disable_modbus_control()  # the axis notes its own deliberate change in the session
        self.assertEqual(self.check().state, rc.LINK_OK)
        self.ax.enable_modbus_control()
        self.assertEqual(self.check().state, rc.LINK_OK)
        self.ax.bus.write_coil(self.ax.slave, R.COIL_PMSL, False)  # dropped behind the axis's back
        self.assertEqual(self.check().state, rc.UNIT_RESTARTED)  # PMSS dropped while the PC held it
        rc.set_link_state(self.ax, rc.LINK_OK)
        rc.note_control(self.ax, modbus_control=False)
        self.assertEqual(self.check().state, rc.LINK_OK)

    def test_home_in_progress_is_not_a_lost_home(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=1)
        ax, _ = open_axis(sim)
        sess = rc.track(ax)
        ax.home(wait=False)
        st = ax.status()
        self.assertTrue(st.homing and not st.homed)
        self.assertEqual(rc.restart_signs(sess, st), [])
        ax.stop()

    def test_recovering_and_untracked(self) -> None:
        self.session.recovering = True
        self.assertEqual(self.check().state, rc.RECOVERING)
        self.session.recovering = False
        other = SimulatedEC(time_scale=50)
        ax, _ = open_axis(other)
        self.assertTrue(rc.check_link(ax).ok)  # no session: the link only
        other.power_off()
        self.assertEqual(rc.check_link(ax).state, rc.UNIT_SILENT)

    def test_link_monitor_reports_changes(self) -> None:
        changes: list[tuple[str, str]] = []
        monitor = rc.LinkMonitor({"G": self.ax}, interval_s=0.01, on_change=lambda n, c: changes.append((n, c.state)))
        with monitor:
            time.sleep(0.05)
            self.sim.power_off()
            deadline = time.perf_counter() + 3.0
            while not changes and time.perf_counter() < deadline:
                time.sleep(0.01)
            self.sim.power_on()
            while len(changes) < 2 and time.perf_counter() < deadline:
                time.sleep(0.01)
        self.assertFalse(monitor.running)
        self.assertEqual(changes[:2], [("G", rc.UNIT_SILENT), ("G", rc.UNIT_RESTARTED)])
        self.assertGreater(monitor.passes, 2)


# ====================================================================== recover()


class RecoverTest(unittest.TestCase):
    def test_power_loss_restores_baud_control_and_safety_and_never_moves(self) -> None:
        sim = SimulatedEC(profile="rod", baudrate=115200, time_scale=50)
        ax, rec = open_axis(sim)
        ax.set_safety_speed(True)
        ax.move_to(80.0, speed_mm_s=100)
        rc.note_target(ax, 80.0)
        time.sleep(0.02)
        rc.track(ax, name="ROD")
        sim.power_cycle()
        events = Events()
        rec.clear()
        report = rc.recover(ax, timeout=5, policy=QUICK, on_event=events)
        self.assertEqual(report.cause, rc.UNIT_SILENT)  # 115200 frames are not understood at 38400
        self.assertTrue(report.restarted and report.same_unit)
        signs = " | ".join(report.restart_signs)
        for needle in ("uptime counter dropped", "power-up record", "PMSS", "HEND", "fell back to 38400"):
            self.assertIn(needle, signs)
        self.assertEqual((report.baudrate_found, report.baudrate, sim.ec_baudrate), (38400, 115200, 115200))
        self.assertTrue(report.baud_restored and report.control_restored and report.safety_speed_restored)
        self.assertFalse(report.control_kept)
        self.assertTrue(sim.pmsl and sim.safety_speed)
        self.assertTrue(report.needs_home and report.needs_attention)
        self.assertFalse(report.ready_to_move)
        self.assertEqual((report.at_target, report.position_error_mm), (True, 0.0))
        self.assertEqual(events.kinds, ["link_lost", "unit_restarted", "recovered"])
        self.assertEqual(motion_frames(rec.tx), [])
        self.assertFalse(sim.moving)
        self.assertEqual(rc.check_link(ax).state, rc.LINK_OK)  # new baseline
        self.assertIn("needs home", report.summary())
        self.assertEqual(report.to_dict()["needs_attention"], True)

    def test_waits_for_the_power_to_come_back(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=50)
        ax, _ = open_axis(sim)
        rc.track(ax)
        sim.power_off()
        # the power comes back 0.3 s after the recovery found the unit silent (a timer started with the call could run
        # out before a busy PC let the recovery look at the unit at all)
        timers: list[threading.Timer] = []

        def power_on_later(event: rc.RecoveryEvent) -> None:
            if event.kind == "link_lost" and not timers:
                timers.append(threading.Timer(0.3, sim.power_on))
                timers[0].start()

        started = time.perf_counter()
        report = rc.recover(ax, timeout=5, policy=QUICK, on_event=power_on_later)
        for timer in timers:
            timer.join()
        self.assertEqual(len(timers), 1)
        self.assertGreaterEqual(time.perf_counter() - started, 0.25)
        self.assertEqual(report.cause, rc.UNIT_SILENT)
        self.assertTrue(report.restarted and report.control_restored)
        self.assertFalse(report.needs_home)  # WA absolute

    def test_power_cycle_after_disable_then_enable_control_takes_pc_control_back(self) -> None:
        # disable_modbus_control() notes "no PC control" in the session; enable_modbus_control() must note it again, or a recover()
        # after a power cycle leaves the unit in PIO mode (no check_link ran between the enable and the power cycle)
        for verify in (True, False):
            with self.subTest(verify=verify):
                sim = SimulatedEC(profile="gripper", time_scale=50)
                ax, _ = open_axis(sim)
                rc.track(ax)
                ax.disable_modbus_control()
                self.assertFalse(sim.pmsl)
                ax.enable_modbus_control(verify=verify)
                self.assertTrue(sim.pmsl)
                sim.power_cycle()  # PMSS = 0 after the power-up
                report = rc.recover(ax, timeout=5, policy=QUICK)
                self.assertTrue(report.restarted)
                self.assertTrue(report.control_restored)
                self.assertFalse(report.control_kept)
                self.assertTrue(sim.pmsl)

    def test_recover_after_disable_then_enable_control_without_a_restart_keeps_pc_control(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=50)
        ax, _ = open_axis(sim)
        rc.track(ax)
        ax.disable_modbus_control()
        ax.enable_modbus_control()
        report = rc.recover(ax, timeout=5, policy=QUICK)
        self.assertTrue(report.control_kept)
        self.assertFalse(report.control_restored)
        self.assertTrue(sim.pmsl)

    def test_port_vanished_and_back_reopens_in_place(self) -> None:
        line = SimLine([SimulatedEC(profile="gripper", time_scale=50)], name="L1")
        ax, _ = open_axis(line)
        rc.track(ax)
        line.unplug()
        threading.Timer(0.2, line.replug).start()
        events = Events()
        report = rc.recover(ax, timeout=5, policy=QUICK, on_event=events)
        self.assertEqual(report.cause, rc.PORT_VANISHED)
        self.assertTrue(report.port_reopened)
        self.assertFalse(report.restarted)  # the unit kept its power
        self.assertEqual(report.actions, ["port L1 reopened"])
        self.assertIn("waited for port L1 to be listed again", report.notes)
        self.assertEqual(line.reopens, 1)
        self.assertEqual(events.kinds, ["link_lost", "recovered"])

    def test_port_that_never_comes_back_times_out(self) -> None:
        sim = SimulatedEC(time_scale=50)
        ax, _ = open_axis(sim)
        rc.track(ax)
        sim.unplug()
        with self.assertRaises(rc.RecoveryError) as caught:
            rc.recover(ax, timeout=0.2, policy=QUICK)
        self.assertIn("did not come back within 0.2 s", str(caught.exception))
        self.assertEqual(caught.exception.report.cause, rc.PORT_VANISHED)
        self.assertEqual(rc.link_state(ax), rc.PORT_VANISHED)

    def test_a_replaced_unit_is_refused_and_nothing_is_written(self) -> None:
        for replacement, field in (({"serial_number": "SIM-0000099"}, "serial_number"),
                                   ({"profile": "slider", "serial_number": "SIM-0000002"}, "model_name")):  # fmt: skip
            with self.subTest(field=field):
                line = SimLine([SimulatedEC(profile="gripper", time_scale=50)], name="L1")
                ax, _ = open_axis(line)
                rc.track(ax)
                line.unplug()
                line.remove(1)
                new = line.add(SimulatedEC(time_scale=50, **{"profile": "gripper", **replacement}))
                line.replug()
                events = Events()
                with self.assertRaises(rc.UnitReplacedError) as caught:
                    rc.recover(ax, timeout=2, policy=QUICK, on_event=events)
                exc = caught.exception
                self.assertIn(field, str(exc))
                self.assertIn("Nothing was written", str(exc))
                self.assertNotEqual(exc.expected[field], exc.found[field])
                self.assertFalse(exc.report.same_unit)
                self.assertLessEqual({f[1] for f in new.requests}, {0x03, 0x11})  # read-only
                self.assertEqual(events.kinds, ["link_lost", "unit_replaced"])
                self.assertEqual(rc.link_state(ax), rc.UNIT_REPLACED)
                self.assertEqual(rc.classify_failure(exc), rc.UNIT_REPLACED)

    def test_needs_a_session_and_an_open_bus(self) -> None:
        sim = SimulatedEC()
        ax, _ = open_axis(sim)
        rc.forget(ax)
        with self.assertRaises(rc.RecoveryError) as caught:
            rc.recover(ax)
        self.assertIn("track(axis)", str(caught.exception))
        rc.track(ax)
        ax.close()
        with self.assertRaises(rc.RecoveryError):
            rc.recover(ax)

    def test_healthy_link_reports_nothing_to_do(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=50)
        ax, rec = open_axis(sim)
        rc.track(ax)
        events = Events()
        rec.clear()
        report = rc.recover(ax, timeout=2, policy=QUICK, on_event=events)
        self.assertEqual(report.cause, rc.LINK_OK)
        self.assertFalse(report.restarted or report.port_reopened or report.needs_attention)
        self.assertEqual(report.actions, [])
        self.assertTrue(report.ready_to_move)
        self.assertEqual(events.items, [])
        self.assertEqual({f[1] for f in rec.tx}, {0x03, 0x11})

    def test_pcon_message_alarm_is_reset_and_comes_back(self) -> None:
        sim = SimulatedEC(profile="pcon", time_scale=50)
        ax, _ = open_axis(sim)
        time.sleep(0.02)
        rc.track(ax)
        sim.power_cycle()
        report = rc.recover(ax, timeout=2, policy=QUICK)
        self.assertEqual(report.cause, rc.UNIT_RESTARTED)
        self.assertEqual(report.alarms_cleared, [R.ALARM_RTC_STOP])
        self.assertEqual(report.alarm_code, R.ALARM_RTC_STOP)
        self.assertFalse(report.fault_alarm)
        self.assertTrue(any("came back after the reset" in n for n in report.notes), report.notes)
        self.assertIn(R.ALARM_RTC_STOP, [r["code"] for r in report.new_alarm_records])

    def test_fault_alarm_is_left_for_the_operator(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=50)
        ax, rec = open_axis(sim)
        rc.track(ax)
        sim.trigger_alarm(0x0C0)
        rec.clear()
        report = rc.recover(ax, timeout=2, policy=QUICK)
        self.assertEqual((report.alarm_code, report.alarms_cleared), (0x0C0, []))
        self.assertTrue(report.fault_alarm and report.needs_attention)
        self.assertTrue(any("left for the operator" in n for n in report.notes))
        self.assertNotIn(R.COIL_ALRS, [struct.unpack(">H", f[2:4])[0] for f in rec.tx if f[1] == 0x05])

    def test_a_move_cut_off_is_reported_not_resumed(self) -> None:
        sim = SimulatedEC(step_s=0.05)  # a stepped clock: the move is still on its way when the power goes
        ax, rec = open_axis(sim)
        rc.track(ax)
        ax.move_to(140.0, speed_mm_s=100, wait=False)
        rc.note_target(ax, 140.0)
        time.sleep(0.05)
        sim.power_cycle()
        rec.clear()
        report = rc.recover(ax, timeout=2, policy=QUICK)
        self.assertFalse(report.at_target)
        self.assertLess(report.position_error_mm, -1.0)
        self.assertTrue(any("NOT resumed" in n for n in report.notes))
        time.sleep(0.05)
        self.assertFalse(sim.moving)
        self.assertEqual(motion_frames(rec.tx), [])

    def test_integrity_of_rows_and_parameters(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=50)
        ax, _ = open_axis(sim)
        ax.write_user_parameter(2, 0.30, commit=False)  # RAM only: lost at power-off
        rc.track(ax, rows=True, parameters=True)
        row = sim.position_row(1)
        row[R.POS_CHECKSUM] ^= 0x55  # a damaged checksum in row 1
        sim.set_position_row(1, row)
        sim.power_cycle()
        policy = rc.RecoveryPolicy(poll_s=0.02, check_rows=True, check_parameters=True)
        report = rc.recover(ax, timeout=2, policy=policy)
        self.assertEqual(report.rows_checked, [0, 1, 2])
        self.assertTrue(any(p.startswith("row 1: checksum") for p in report.row_problems), report.row_problems)
        self.assertGreater(report.parameters_checked, 50)
        self.assertEqual(len(report.parameter_differences), 1)
        self.assertIn("8.9", report.parameter_differences[0])
        self.assertTrue(report.needs_attention)

    def test_integrity_against_a_backup(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=50)
        ax, _ = open_axis(sim)
        ax.write_user_parameter(2, 0.30, commit=False)
        saved = backup(ax, rows=[0, 1, 2])
        rc.track(ax)
        sim.power_cycle()
        policy = rc.RecoveryPolicy(poll_s=0.02, check_rows=True, check_parameters=True, backup=saved)
        report = rc.recover(ax, timeout=2, policy=policy)
        self.assertEqual(report.row_problems, [])
        self.assertEqual([d.split(":")[1] for d in report.parameter_differences], ["8.9"])
        other = dict(saved, identity={**saved["identity"], "serial_number": "ELSEWHERE"})
        report = rc.recover(ax, timeout=2, policy=rc.RecoveryPolicy(poll_s=0.02, check_parameters=True, backup=other))
        self.assertTrue(any("another unit" in n for n in report.notes), report.notes)
        self.assertEqual(report.parameter_differences, [])

    def test_policy_can_leave_the_session_alone(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=50)
        ax, _ = open_axis(sim)
        ax.set_safety_speed(True)
        rc.track(ax)
        sim.power_cycle()
        policy = rc.RecoveryPolicy(poll_s=0.02, restore_control=False, restore_safety_speed=False)
        report = rc.recover(ax, timeout=2, policy=policy)
        self.assertFalse(report.control_restored or report.safety_speed_restored)
        self.assertFalse(report.control_kept)  # control was lost and the policy leaves it: neither
        self.assertEqual(report.actions, [])
        self.assertFalse(sim.pmsl or sim.safety_speed)
        self.assertTrue(rc.check_link(ax).ok)  # the new baseline is what the unit has now
        ax.enable_modbus_control()
        self.assertTrue(rc.check_link(ax).ok)  # a healthy check moves the baseline forward: PC control held again
        rc.note_control(ax, safety_speed=True)  # the application's choice, restored next time
        sim.power_cycle()
        report = rc.recover(ax, timeout=2, policy=QUICK)
        self.assertTrue(report.control_restored and report.safety_speed_restored)
        self.assertTrue(sim.pmsl and sim.safety_speed)

    def test_an_unverified_rate_is_not_restored(self) -> None:
        sim = SimulatedEC(profile="gripper", baudrate=57600, time_scale=50)
        ax, _ = open_axis(sim)
        rc.track(ax)
        sim.power_cycle()
        report = rc.recover(ax, timeout=3, policy=QUICK)
        self.assertEqual((report.baudrate_found, report.baudrate, report.baud_restored), (38400, 38400, False))
        self.assertEqual(sum("no verified baud-change code" in n for n in report.notes), 1, report.notes)  # tried once

    def test_line_break_back_to_38400_before_the_full_identify(self) -> None:
        sim = SimulatedEC(profile="gripper", baudrate=38400, time_scale=50, ignored_baud_codes=())  # takes 0x06 (VERIFIED: gripper)
        ax, rec = open_axis(sim)
        rc.track(ax)
        ax.bus.send_break(0.25)  # the unit drops to 9600
        rec.clear()
        report = rc.recover(ax, timeout=5, policy=QUICK)
        self.assertEqual((report.cause, report.restarted, report.same_unit), (rc.UNIT_SILENT, False, True))
        self.assertEqual((report.baudrate_found, report.baudrate, report.baud_restored), (9600, 38400, True))
        baud = next(i for i, f in enumerate(rec.tx) if f[:4] == bytes([0x00, 0x45, 0x00, 0x04]))
        directory = next(i for i, f in enumerate(rec.tx) if f[1] == 0x03 and f[2:4] == b"\x00\x00")
        self.assertLess(baud, directory)  # the many identify() reads run at 38400, not 9600
        self.assertEqual(sum(a.startswith("baud rate restored") for a in report.actions), 1, report.actions)

    def test_a_line_break_keeps_pc_control_and_reports_it_as_kept(self) -> None:
        sim = SimulatedEC(profile="gripper", baudrate=38400, time_scale=50, ignored_baud_codes=())
        ax, _ = open_axis(sim)
        rc.track(ax)
        ax.bus.send_break(0.25)  # the unit drops to 9600 but keeps PMSS
        report = rc.recover(ax, timeout=5, policy=QUICK)
        self.assertEqual((report.restarted, report.modbus_control), (False, True))
        self.assertEqual((report.control_kept, report.control_restored), (True, False))
        self.assertFalse(any("PC control" in a for a in report.actions), report.actions)
        self.assertTrue(report.to_dict()["control_kept"])
        self.assertTrue(sim.pmsl)

    def test_a_unit_that_ignores_the_38400_code_stays_reachable(self) -> None:
        sim = SimulatedEC(profile="gripper", baudrate=38400, time_scale=50)  # ignores 0x06, as the EC-R6S rod did
        ax, rec = open_axis(sim)
        rc.track(ax)
        ax.bus.send_break(0.25)
        rec.clear()
        report = rc.recover(ax, timeout=5, policy=QUICK)
        self.assertTrue(report.same_unit and report.ready_to_move)
        self.assertEqual((report.baudrate_found, report.baudrate, report.baud_restored), (9600, 9600, False))
        self.assertEqual(sum("not restored" in n for n in report.notes), 1, report.notes)  # tried once, noted once
        self.assertEqual(len([f for f in rec.tx if f[:4] == bytes([0x00, 0x45, 0x00, 0x04])]), 1)  # one broadcast

    def test_session_from_info_without_a_baseline(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=50)
        ax, _ = open_axis(sim)
        rc.forget(ax)
        sess = rc.session_from_info(ax, ax.identify(), name="G")
        self.assertIs(rc.session_of(ax), sess)
        sim.power_cycle()
        report = rc.recover(ax, timeout=2, policy=QUICK)
        self.assertTrue(report.same_unit)
        self.assertFalse(report.control_restored or report.control_kept)  # not known before the fault: neither
        self.assertTrue(any("unknown (no track())" in n for n in report.notes), report.notes)
        self.assertIsInstance(sess.to_dict(), dict)


# ====================================================================== Cell


class CellRecoveryTest(unittest.TestCase):
    def test_a_vanished_port_stops_the_others_and_recover_all_brings_it_back(self) -> None:
        ports = sim_ports(3, step_s=0.05)  # a stepped clock: no axis reaches its end before the stop, whatever the load
        faults: list[tuple[str, str]] = []
        with Cell.open(ports, on_fault=lambda n, k, e: faults.append((n, k)), **OPEN) as cell:
            cell.track_all()
            cell.move_all({n: 140.0 for n in cell.names}, speed_mm_s=60, wait=False)
            time.sleep(0.1)
            ports["SIM2"].unplug()
            with self.assertRaises(CellError) as caught:
                cell.wait_all()
            self.assertIsInstance(caught.exception.errors["SIM2:1"], TransportError)
            self.assertEqual(faults, [("SIM2:1", rc.PORT_VANISHED)])
            self.assertEqual(cell.faults, {"SIM2:1": rc.PORT_VANISHED})
            for name in ("SIM1", "SIM3"):
                self.assertFalse(ports[name][1].moving)
                self.assertLess(ports[name][1].position_mm, 139.0)
            ports["SIM2"].replug()
            ports["SIM3"].power_cycle()
            mark = {n: len(line.requests) for n, line in ports.items()}
            reports = cell.recover_all(timeout=3, policy=QUICK)
            self.assertEqual(cell.faults, {})
            self.assertTrue(reports["SIM2:1"].port_reopened)
            self.assertTrue(reports["SIM3:1"].restarted)
            self.assertFalse(reports["SIM1:1"].restarted)
            self.assertFalse(reports["SIM1:1"].at_target)  # stopped short: reported, not resumed
            # the ports recover at the same time: the order holds per axis, not across them
            events = [(e.kind, e.axis) for e in cell.events]
            self.assertEqual(events[0], ("link_lost", "SIM2:1"))
            self.assertEqual(sorted(events), sorted([("link_lost", "SIM2:1"), ("unit_restarted", "SIM3:1"),
                                                     ("recovered", "SIM2:1"), ("recovered", "SIM3:1")]))  # fmt: skip
            for axis in ("SIM2:1", "SIM3:1"):
                self.assertEqual([k for k, a in events if a == axis][-1], "recovered", axis)
            for name, line in ports.items():
                self.assertEqual(motion_frames([f for _, f in line.requests[mark[name]:]]), [], name)

    def test_stop_on_fault_can_be_switched_off(self) -> None:
        ports = sim_ports(2, time_scale=2)
        with Cell.open(ports, stop_on_fault=False, **OPEN) as cell:
            cell.move_all({n: 140.0 for n in cell.names}, speed_mm_s=60, wait=False)
            time.sleep(0.05)
            ports["SIM1"].unplug()
            res = cell.status_all(return_exceptions=True)
            self.assertIsInstance(res["SIM1:1"], TransportError)
            self.assertEqual(cell.faults, {"SIM1:1": rc.PORT_VANISHED})
            self.assertTrue(ports["SIM2"][1].moving)  # left running
            cell.stop_all(["SIM2:1"])

    def test_check_links_tracks_and_detects_a_restart(self) -> None:
        ports = sim_ports(2, time_scale=50)
        seen: list[rc.RecoveryEvent] = []
        with Cell.open(ports, on_event=seen.append, **OPEN) as cell:
            time.sleep(0.02)
            checks = cell.check_links()
            self.assertTrue(all(c.ok for c in checks.values()))
            self.assertIsNotNone(rc.session_of(cell["SIM1:1"]))
            ports["SIM1"].power_cycle()
            checks = cell.check_links()
            self.assertEqual(checks["SIM1:1"].state, rc.UNIT_RESTARTED)
            self.assertTrue(checks["SIM2:1"].ok)
            self.assertEqual(cell.faults, {"SIM1:1": rc.UNIT_RESTARTED})
            self.assertEqual([e.kind for e in seen], ["unit_restarted"])
            report = cell.recover_all(["SIM1:1"], timeout=2, policy=QUICK)["SIM1:1"]
            self.assertTrue(report.restarted and report.control_restored)
            self.assertEqual([e.kind for e in seen], ["unit_restarted", "recovered"])
            self.assertTrue(all(c.ok for c in cell.check_links().values()))

    def test_shared_line_power_loss_restores_the_baud_rate_once(self) -> None:
        line = SimLine([SimulatedEC(slave=s, baudrate=115200, time_scale=50) for s in (1, 2)], name="L")
        with Cell.open([line], **OPEN) as cell:
            cell.track_all()
            line.power_cycle()
            self.assertEqual([u.ec_baudrate for u in line], [38400, 38400])
            reports = cell.recover_all(timeout=5, policy=QUICK)
            self.assertTrue(reports["L:1"].baud_restored)
            self.assertFalse(reports["L:2"].baud_restored)  # the broadcast already switched it
            self.assertEqual([u.ec_baudrate for u in line], [115200, 115200])
            self.assertTrue(all(r.restarted and r.control_restored for r in reports.values()))
            self.assertTrue(all(u.pmsl for u in line))

    def test_recover_all_refuses_a_replaced_unit(self) -> None:
        ports = sim_ports(2, time_scale=50)
        with Cell.open(ports, **OPEN) as cell:
            line = ports["SIM2"]
            line.unplug()
            line.remove(1)
            line.add(SimulatedEC(time_scale=50, serial_number="OTHER"))
            line.replug()
            with self.assertRaises(CellError) as caught:
                cell.recover_all(timeout=2, policy=QUICK)  # never tracked: compared with Cell.open's identity
            self.assertEqual(list(caught.exception.errors), ["SIM2:1"])
            self.assertIsInstance(caught.exception.errors["SIM2:1"], rc.UnitReplacedError)
            self.assertEqual(cell.faults, {"SIM2:1": rc.UNIT_REPLACED})
            self.assertIn("unit_replaced", [e.kind for e in cell.events])

    def test_monitor_links_reports_a_fault_and_stops_with_the_cell(self) -> None:
        ports = sim_ports(2, time_scale=2)
        faults: list[tuple[str, str]] = []
        cell = Cell.open(ports, on_fault=lambda n, k, e: faults.append((n, k)), **OPEN)
        try:
            monitor = cell.monitor_links(0.02)
            cell.move_all({"SIM2:1": 140.0}, speed_mm_s=60, wait=False)
            ports["SIM1"].unplug()
            deadline = time.perf_counter() + 3.0
            while not faults and time.perf_counter() < deadline:
                time.sleep(0.01)
            self.assertEqual(faults, [("SIM1:1", rc.PORT_VANISHED)])
            while ports["SIM2"][1].moving and time.perf_counter() < deadline:
                time.sleep(0.01)
            self.assertFalse(ports["SIM2"][1].moving)  # stopped by the fault handling
            self.assertLess(ports["SIM2"][1].position_mm, 139.0)
            def vanished() -> bool:  # stored once the stop it triggered has finished (a check before the unplug: ok)
                check = monitor.states.get("SIM1:1")
                return check is not None and check.state == rc.PORT_VANISHED

            while not vanished() and time.perf_counter() < deadline:
                time.sleep(0.01)
            self.assertEqual(monitor.states["SIM1:1"].state, rc.PORT_VANISHED)
        finally:
            cell.close()
        self.assertFalse(monitor.running)


# ====================================================================== concurrent calls, their results and events


class _Refusing(SimulatedEC):
    """A unit that answers the status block (FC03 at 0x9000) with exception 02 while ``refuse`` is set."""

    refuse = False

    def on_request(self, frame: bytes) -> bytes | None:
        if self.refuse and len(frame) >= 4 and frame[1] == 0x03 and struct.unpack(">H", frame[2:4])[0] == R.PNOW:
            raise SimException(0x02)
        return NotImplemented  # type: ignore[return-value]


class _Asserting(SimulatedEC):
    """A test unit whose request check fails (an assert in ``on_request``) while ``fail`` is set."""

    fail = False

    def on_request(self, frame: bytes) -> bytes | None:
        if self.fail and frame[1] == 0x03:
            raise AssertionError("unexpected frame")
        return NotImplemented  # type: ignore[return-value]


class _Checking:
    """check_link (or ``call``) on another thread, back to back, until stopped."""

    def __init__(self, call: object) -> None:
        self.call = call
        self.runs = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self.call()  # type: ignore[operator]
            except Exception:
                pass
            self.runs += 1

    def __enter__(self) -> _Checking:
        self._thread.start()
        deadline = time.perf_counter() + 2.0
        while self.runs < 2 and time.perf_counter() < deadline:
            time.sleep(0.002)
        return self

    def __exit__(self, *exc: object) -> None:
        self._stop.set()
        self._thread.join(5.0)


class ConcurrentCallsTest(unittest.TestCase):
    def test_recover_returns_its_own_report_while_another_thread_checks_the_link(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=50)
        ax, _ = open_axis(sim)
        rc.track(ax, name="G")
        with _Checking(lambda: rc.check_link(ax)):
            for _ in range(4):
                report = rc.recover(ax, timeout=2, policy=QUICK, on_event=lambda e: None)
                self.assertTrue(report.same_unit)
                self.assertTrue(report.identity.get("serial_number"))
                report = rc.recover(ax, timeout=2, policy=QUICK)
                self.assertTrue(report.same_unit)
        ax.close()

    def test_run_check_returns_its_own_report_while_another_thread_checks_the_link(self) -> None:
        sim = SimulatedEC(time_scale=50)
        ax, _ = open_axis(sim)
        lines: list[str] = []

        def on_item(item: object) -> None:  # a slow reader of the lines: the test ends while it reads
            lines.append(item.name)  # type: ignore[attr-defined]
            time.sleep(0.01)

        with _Checking(lambda: rc.check_link(ax)):
            for _ in range(3):
                lines.clear()
                report = run_check(ax, port_label="sim", on_item=on_item)
                self.assertTrue(report.items)
                self.assertEqual(report.items[0].name, "link")
                self.assertEqual([i.name for i in report.items], lines)
        ax.close()

    def test_recover_all_returns_its_reports_while_another_thread_checks_the_links(self) -> None:
        ports = sim_ports(2, time_scale=50)
        with Cell.open(ports, **OPEN) as cell:
            cell.track_all()
            with _Checking(cell.check_links):
                for _ in range(10):
                    reports = cell.recover_all(timeout=2, policy=QUICK)
                    self.assertEqual(sorted(reports), ["SIM1:1", "SIM2:1"])
                    self.assertTrue(all(r is not None and r.same_unit for r in reports.values()), reports)

    def test_each_call_with_on_event_gets_its_own_events_while_a_link_monitor_runs(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=50)
        ax, _ = open_axis(sim)
        rc.track(ax, name="G")
        logged = Events()
        with rc.LinkMonitor({"G": ax}, interval_s=0.001, on_event=logged):
            for _ in range(3):
                sim.power_off()
                time.sleep(0.05)
                sim.power_on()
                seen = Events()
                rc.recover(ax, timeout=3, policy=QUICK, on_event=seen)
                self.assertIn("recovered", seen.kinds)
        self.assertNotIn("recovered", logged.kinds)  # a recovery's events are never the monitor's
        ax.close()

    def test_an_assert_in_on_request_reaches_the_caller_of_check_link(self) -> None:
        sim = _Asserting(time_scale=50)
        ax, _ = open_axis(sim)
        rc.track(ax, name="A")
        sim.fail = True
        with self.assertRaisesRegex(AssertionError, "unexpected frame"):
            rc.check_link(ax)
        sim.fail = False
        self.assertTrue(rc.check_link(ax).ok)
        ax.close()

    def test_run_check_on_a_closed_axis_reports_the_link_failure(self) -> None:
        ax = Elecylinder.open(SimulatedEC(), **FAST)  # type: ignore[arg-type]
        ax.close()
        report = run_check(ax, port_label="sim")
        self.assertEqual((report.items[0].name, report.items[0].result), ("link", FAIL))
        self.assertIn("bus is closed", report.items[0].detail)
        self.assertFalse(report.passed)
        self.assertTrue(all(i.result == SKIP for i in report.items[1:]), [(i.name, i.result) for i in report.items])


class KeptResultTest(unittest.TestCase):
    """A call on another thread replaces the handle's last result just as the call under test returns (each test makes
    it happen at that very moment): the call gives its own result."""

    @staticmethod
    def _after(function: str, then: object) -> object:
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

    def test_recover_gives_its_own_report(self) -> None:
        ax, _ = open_axis(SimulatedEC(profile="gripper", time_scale=50))
        rc.track(ax, name="G")
        with self._after("iad_axis_recover", lambda: rc.check_link(ax)):
            for on_event in (None, lambda e: None):
                report = rc.recover(ax, timeout=2, policy=QUICK, on_event=on_event)
                self.assertTrue(report.same_unit)
                self.assertTrue(report.identity.get("serial_number"))
        ax.close()

    def test_run_check_gives_its_own_report(self) -> None:
        ax, _ = open_axis(SimulatedEC(time_scale=50))
        with self._after("iad_axis_self_test", lambda: rc.check_link(ax)):
            for on_item in (None, lambda item: None):
                report = run_check(ax, port_label="sim", on_item=on_item)
                self.assertEqual(report.items[0].name, "link")
                self.assertTrue(report.passed)
        ax.close()

    def test_recover_all_gives_its_own_reports(self) -> None:
        with Cell.open(sim_ports(2, time_scale=50), **OPEN) as cell:
            cell.track_all()
            with self._after("iad_cell_recover_all", cell.check_links):
                reports = cell.recover_all(timeout=2, policy=QUICK)
            self.assertEqual(sorted(reports), ["SIM1:1", "SIM2:1"])
            self.assertTrue(all(r is not None and r.same_unit for r in reports.values()), reports)

    def test_events_are_in_cell_events_when_a_slow_event_thread_is_flushed(self) -> None:
        ports = sim_ports(2, time_scale=50)
        real = Cell._dispatch

        def slow(self: Cell, e: object) -> None:  # the event thread is late with every event
            time.sleep(0.3)
            real(self, e)  # type: ignore[arg-type]

        with mock.patch.object(Cell, "_dispatch", slow), Cell.open(ports, **OPEN) as cell:  # patched before the thread
            cell.check_links()
            ports["SIM1"].unplug()
            cell.check_links()
            self.assertEqual([e.kind for e in cell.events], ["link_lost"])


class CellEventsTest(unittest.TestCase):
    def test_events_are_in_cell_events_when_check_links_returns_without_callbacks(self) -> None:
        ports = sim_ports(2, time_scale=50)
        with Cell.open(ports, **OPEN) as cell:
            cell.check_links()
            ports["SIM1"].unplug()
            cell.check_links()
            self.assertEqual([(e.kind, e.axis) for e in cell.events], [("link_lost", "SIM1:1")])
            ports["SIM1"].replug()
            cell.recover_all(["SIM1:1"], timeout=2, policy=QUICK)
            self.assertEqual([e.kind for e in cell.events], ["link_lost", "recovered"])

    def test_a_link_monitors_on_change_can_recover_the_cell(self) -> None:
        ports = sim_ports(2, time_scale=50)
        with Cell.open(ports, **OPEN) as cell:
            cell.track_all()
            done: list[tuple[list[str], float]] = []

            def on_change(name: str, chk: rc.LinkCheck) -> None:
                if chk.state == rc.UNIT_RESTARTED and not done:
                    start = time.perf_counter()
                    cell.recover_all([name], timeout=2, policy=QUICK)
                    done.append(([e.kind for e in cell.events], time.perf_counter() - start))

            monitor = cell.monitor_links(0.02, on_change=on_change)
            ports["SIM1"].power_cycle()
            deadline = time.perf_counter() + 5.0
            while not done and time.perf_counter() < deadline:
                time.sleep(0.01)
            monitor.stop()
            self.assertTrue(done)
            kinds, elapsed = done[0]
            self.assertIn("recovered", kinds)  # delivered before recover_all returned, on the callback's thread too
            self.assertLess(elapsed, 2.0)

    def test_check_now_of_a_cell_link_monitor_updates_it_and_calls_on_change(self) -> None:
        ports = sim_ports(2, time_scale=50)
        changes: list[tuple[str, str]] = []
        with Cell.open(ports, **OPEN) as cell:
            monitor = cell.monitor_links(3600, on_change=lambda name, chk: changes.append((name, chk.state)))
            deadline = time.perf_counter() + 5.0
            while monitor.passes < 1 and time.perf_counter() < deadline:
                time.sleep(0.01)
            self.assertEqual(monitor.passes, 1)
            ports["SIM1"].unplug()
            checks = monitor.check_now()
            self.assertEqual(checks["SIM1:1"].state, rc.PORT_VANISHED)
            self.assertTrue(checks["SIM2:1"].ok)
            self.assertEqual(changes, [("SIM1:1", rc.PORT_VANISHED)])  # before check_now returned
            self.assertEqual(monitor.states["SIM1:1"].state, rc.PORT_VANISHED)
            self.assertEqual(monitor.passes, 2)
            monitor.stop()
            monitor.check_now()  # stopped: the same rule over the states it kept
            self.assertEqual(changes, [("SIM1:1", rc.PORT_VANISHED)])
            self.assertEqual(monitor.passes, 3)
            self.assertEqual(monitor.states["SIM1:1"].state, rc.PORT_VANISHED)

    def test_check_links_gives_a_failure_that_is_not_a_link_fault_as_its_own_exception(self) -> None:
        unit = _Refusing(time_scale=50)
        with Cell.open([SimLine([unit], name="L")], **OPEN) as cell:
            self.assertTrue(cell.check_links()["L:1"].ok)
            unit.refuse = True
            result = cell.check_links()["L:1"]
            self.assertIsInstance(result, ModbusException)
            self.assertEqual((result.code, result.function), (0x02, 0x03))  # type: ignore[union-attr]
            self.assertEqual(cell.faults, {})


if __name__ == "__main__":
    unittest.main()
