"""Axis layer: PPOW truncation, the FC06 "register" strategy and coil-free STOP, the 22-word status and its
condition flags, servo-ON reasons and CON timeout, automatic servo OFF, mid-move speed / accel changes, cancelling a
paused move, teaching, acceleration patterns and the unit's push speed (simulator only)."""

from __future__ import annotations

import struct
import time
import unittest
from dataclasses import replace
from types import SimpleNamespace

from ec_testutil import FrameRecorder, SteppedClockSim, hexbytes, make_ec

from iai_actuator_driver import registers as R
from iai_actuator_driver.alarms import LEVEL_COLD_START, alarm_level
from iai_actuator_driver.ec import Elecylinder, RegisterCommandStrategy, Status
from iai_actuator_driver.errors import (
    EcAlarmError,
    EcNotReadyError,
    LineRuleError,
    ModbusException,
    ServoBlockedError,
    SoftLimitError,
    UnsafeCommandError,
)
from iai_actuator_driver.modbus import ModbusRTU, build_frame
from iai_actuator_driver.sim import SimException, SimulatedEC


# DRG1 frames for slave 01 (servo on, safety speed off), CRC included
SERVO_OFF = hexbytes("01 06 0D 00 00 00 8B 66")
SERVO_ON = hexbytes("01 06 0D 00 10 00 86 A6")
HOME_EDGE = hexbytes("01 06 0D 00 10 10 87 6A")
ALARM_RESET = hexbytes("01 06 0D 00 11 00 87 36")
PAUSE = hexbytes("01 06 0D 00 10 20 87 7E")
CANCEL_PAUSED = hexbytes("01 06 0D 00 11 20 86 EE")


def direct_moves(frames: list[bytes]) -> list[tuple[int, ...]]:
    """The 9-word 0x9900 writes as (pcmd, inp, vcmd, acmd, ppow, ctlf)."""
    out = []
    for f in frames:
        if len(f) > 25 and f[1] == 0x10 and struct.unpack(">HH", f[2:6]) == (R.DIRECT_MOVE_BASE, 9):
            w = struct.unpack(">9H", f[7:25])
            out.append((R.from_i32_words(w[0], w[1]), R.from_u32_words(w[2], w[3]),
                        R.from_u32_words(w[4], w[5]), w[6], w[7], w[8]))  # fmt: skip
    return out


def drg1_writes(frames: list[bytes]) -> list[bytes]:
    """Every FC06 write to DRG1 0x0D00, as the full frame (CRC included)."""
    return [f for f in frames if f[1] == 0x06 and f[2:4] == b"\x0d\x00"]


def coil_writes(frames: list[bytes]) -> list[bytes]:
    return [f for f in frames if f[1] == 0x05]


def wait_for(predicate, timeout: float = 3.0) -> bool:  # type: ignore[no-untyped-def]
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


def open_ec(sim: SimulatedEC, **kwargs: object) -> tuple[Elecylinder, FrameRecorder]:
    rec = FrameRecorder()
    kwargs.setdefault("poll_interval_s", 0.005)
    ec = Elecylinder(ModbusRTU(sim, frame_logger=rec, min_gap_s=0.0), owns_bus=True, **kwargs)  # type: ignore[arg-type]
    return ec, rec


# =========================================================================== PPOW


class PushPowerTest(unittest.TestCase):
    """PPOW = floor(pct x 255 / 100) everywhere (DOC ME0162-12A p.5-132, 5-142): 70 % -> 0xB2, never 0xB3."""

    def test_table_1_to_100_percent(self) -> None:
        for pct in range(1, 101):
            with self.subTest(pct=pct):
                self.assertEqual(R.push_power(pct), pct * 255 // 100)
                self.assertEqual(R.push_power(float(pct)), pct * 255 // 100)
        self.assertEqual([R.push_power(p) for p in (20, 30, 50, 60, 70)], [0x33, 0x4C, 0x7F, 0x99, 0xB2])
        self.assertEqual((R.push_power(100), R.push_power(200)), (0xFF, R.PPOW_RAW_MAX))
        self.assertEqual(R.push_power(33.3), 84)  # 84.915 truncated
        self.assertEqual(R.push_power(70), R.PPOW_MAX)
        self.assertAlmostEqual(R.push_pct(0xB2), 69.8, places=1)
        with self.assertRaises(ValueError):
            R.push_power(float("nan"))

    def test_push_sends_0x7f_for_50_percent(self) -> None:
        ec, sim, _ = make_ec(profile="rod")
        with ec:
            ec.push(80.0, 10.0, 50.0)
        writes = [f for f in sim.requests if f[1] == 0x10 and struct.unpack(">H", f[2:4])[0] == R.DIRECT_MOVE_BASE]
        words = struct.unpack(">9H", writes[-1][7:25])
        self.assertEqual(words[R.PPOW - R.DIRECT_MOVE_BASE], 0x007F)

    def test_rod_row_at_70_percent_stores_0xb2_and_refuses_0xb3(self) -> None:
        ec, sim, _ = make_ec(profile="rod", max_speed_mm_s=800.0, max_accel_g=1.0)  # the rod's own row values
        with ec:
            self.assertEqual(ec.push_cap_pct, 70.0)  # the rod's MXPP
            row = ec.read_position_table(1)
            self.assertTrue(row.control_flags & R.CTLF_PUSH)
            written = ec.write_position_table(1, replace(row, push_power=0xB2), True)
            self.assertEqual((written.push_power, written.checksum_ok), (0xB2, True))
            writes = sim.nonvolatile_writes
            with self.assertRaisesRegex(SoftLimitError, "outside 1..178"):
                ec.write_position_table(1, replace(row, push_power=0xB3), True)
            self.assertEqual(sim.nonvolatile_writes, writes)

    def test_stored_row_above_the_cap_in_counts_is_not_run(self) -> None:
        ec, sim, rec = make_ec(profile="rod")
        with ec:
            row = sim.position_row(1)
            row[R.POS_PPOW] = 0xB3  # 70.2 %: one count above floor(70 x 255 / 100)
            row[R.POS_CHECKSUM] = R.position_checksum(row[:15])
            sim.set_position_row(1, row)
            with self.assertRaisesRegex(SoftLimitError, "push force"):
                ec.move_to_position(1, allow_above_caps=True)  # cap = MXPP 70 % = 0xB2 counts
            self.assertFalse([f for f in rec.tx if f[1] == 0x06 and f[2:4] == b"\x98\x00"])


# =========================================================================== register strategy


class RegisterStrategyTest(unittest.TestCase):
    """FC06 DRG1 writes keeping SON (DOC ME0162-12A p.5-124; frames with their CRC)."""

    def test_frames_match_the_documented_ones(self) -> None:
        cases = {
            "servo on": (0x1000, "01 06 0D 00 10 00 86 A6"),
            "servo off": (0x0000, "01 06 0D 00 00 00 8B 66"),
            "home": (0x1010, "01 06 0D 00 10 10 87 6A"),
            "alarm reset": (0x1100, "01 06 0D 00 11 00 87 36"),
            "pause": (0x1020, "01 06 0D 00 10 20 87 7E"),
            "cancel while paused": (0x1120, "01 06 0D 00 11 20 86 EE"),
        }
        for name, (value, frame) in cases.items():
            with self.subTest(name):
                self.assertEqual(build_frame(1, struct.pack(">BHH", 0x06, R.DRG1, value)), hexbytes(frame))

    def test_servo_home_reset_pause_through_fc06(self) -> None:
        ec, sim, rec = make_ec(strategy="register")
        with ec:
            self.assertEqual((ec.strategy.name, ec.stop_path), ("register", "register"))
            ec.servo(False)
            ec.servo(True)
            self.assertEqual(drg1_writes(rec.tx), [SERVO_OFF, SERVO_ON])
            rec.clear()
            st = ec.home()
            self.assertTrue(st.homed)
            self.assertEqual(drg1_writes(rec.tx), [HOME_EDGE, SERVO_ON])
            rec.clear()
            sim.trigger_alarm(0x0E0)
            st = ec.reset_alarm()
            self.assertEqual(st.alarm_code, 0)
            alarm_reset = build_frame(1, struct.pack(">BHH", 0x06, R.DRG1, R.DRG1_ALRS))  # the alarm dropped the servo
            self.assertEqual(drg1_writes(rec.tx)[0], alarm_reset)
            ec.servo(True)
            rec.clear()
            ec.reset_alarm()  # with the servo on: SON is kept in both frames
            self.assertEqual(drg1_writes(rec.tx), [ALARM_RESET, SERVO_ON])
            rec.clear()
            ec.pause()
            self.assertTrue(ec.status().paused)
            ec.resume()
            self.assertEqual(drg1_writes(rec.tx), [PAUSE, SERVO_ON])
            self.assertEqual(coil_writes(rec.tx), [])

    def test_safety_speed_is_kept_in_every_write(self) -> None:
        ec, sim, rec = make_ec(strategy="register")
        with ec:
            ec.set_safety_speed(True)
            self.assertTrue(sim.safety_speed)
            rec.clear()
            ec.pause()
            ec.resume()
            self.assertEqual([struct.unpack(">H", f[4:6])[0] for f in drg1_writes(rec.tx)], [0x5020, 0x5000])
            self.assertEqual(coil_writes(rec.tx), [])

    def test_stop_substitute_ends_at_standstill_without_resuming(self) -> None:
        ec, sim, rec = make_ec(strategy="register", time_scale=5.0)
        with ec:
            ec.move_to(140.0, speed_mm_s=20.0, wait=False)
            self.assertTrue(wait_for(lambda: sim.position_mm > 75.0))
            rec.clear()
            st = ec.stop()
            self.assertFalse(st.moving)
            self.assertEqual(
                drg1_writes(rec.tx),
                [PAUSE, CANCEL_PAUSED,
                 PAUSE, SERVO_ON],
            )  # fmt: skip
            self.assertEqual(coil_writes(rec.tx), [])
            stopped = sim.position_mm
            time.sleep(0.3)  # 1.5 s simulated: the old target must not be resumed
            self.assertAlmostEqual(sim.position_mm, stopped, places=2)
            self.assertFalse(sim.moving)
            self.assertLess(stopped, 140.0)
            final = ec.status()
            self.assertTrue(final.in_position and not final.paused)  # PEND back once the pause is released

    def test_begin_stop_then_stop(self) -> None:
        ec, sim, rec = make_ec(strategy="register", time_scale=5.0)
        with ec:
            ec.move_to(140.0, speed_mm_s=20.0, wait=False)
            rec.clear()
            ec.begin_stop()
            self.assertEqual(drg1_writes(rec.tx), [PAUSE])
            ec.stop()
            self.assertFalse(sim.moving)
            self.assertFalse(ec.status().paused)

    def test_error_path_uses_the_substitute(self) -> None:
        ec, sim, rec = make_ec(strategy="register", time_scale=5.0)
        with ec:
            ec.move_to(140.0, speed_mm_s=20.0, wait=False)
            with self.assertRaises(Exception):
                ec.wait_idle(target_mm=140.0, timeout=0.05)  # MotionTimeoutError: the guard stops the axis
            self.assertFalse(sim.moving)
            self.assertEqual(coil_writes(rec.tx), [])
            self.assertFalse(ec.status().paused)

    def test_fc05_exception_01_switches_to_register(self) -> None:
        sim = SimulatedEC(time_scale=50.0, function_codes=(0x03, 0x06, 0x10, 0x11, 0x17, 0x45))
        ec, rec = open_ec(sim, stroke=(0.0, 150.0))
        with ec:
            with self.assertLogs("iai_actuator_driver.ec", "WARNING"):
                ec.servo(False)
            self.assertFalse(sim.servo_on)
            self.assertEqual((ec.strategy.name, ec.stop_path), ("register", "register"))
            ec.servo(True)
            ec.move_to(140.0, speed_mm_s=50.0, wait=False)
            rec.clear()
            ec.stop()
            self.assertEqual(coil_writes(rec.tx), [])
            self.assertFalse(sim.moving)

    def test_stop_coil_refused_switches_the_stop_path(self) -> None:
        sim = SimulatedEC(time_scale=5.0, function_codes=(0x03, 0x06, 0x10, 0x11, 0x17))
        ec, rec = open_ec(sim, stroke=(0.0, 150.0))
        with ec:
            ec.move_to(140.0, speed_mm_s=20.0, wait=False)
            with self.assertLogs("iai_actuator_driver.ec", "WARNING"):
                ec.stop()
            self.assertEqual(ec.stop_path, "register")
            self.assertFalse(sim.moving)

    def test_explicit_register_method(self) -> None:
        ec, sim, rec = make_ec()
        with ec:
            sim.trigger_alarm(0x0E0)
            ec.reset_alarm(method="register")
            self.assertEqual(sim.alarm_code, 0)
            self.assertEqual(coil_writes(rec.tx), [])
            self.assertEqual(ec.strategy.name, "coil")  # a per-call method does not change the strategy

    def test_image_read_failure_reuses_the_last_image_only(self) -> None:
        strategy = RegisterCommandStrategy()

        class Bus:
            def __init__(self) -> None:
                self.writes: list[int] = []
                self.fail = False

            def read_registers(self, slave: int, address: int, count: int) -> list[int]:
                if self.fail:
                    raise ModbusException(0x04, 0x03)
                return [0x5000 | R.DRG1_ALRS]

            def write_register(self, slave: int, address: int, value: int) -> None:
                self.writes.append(value)

        bus = Bus()
        bus.fail = True
        # no image and DSS1 unreadable: a pause (first frame of a STOP) is still sent, with SON alone kept
        strategy.pause(bus, 1, True)  # type: ignore[arg-type]
        bus.fail = False
        strategy.pause(bus, 1, True)  # type: ignore[arg-type]
        bus.fail = True
        strategy.pause(bus, 1, False)  # type: ignore[arg-type]
        self.assertEqual(bus.writes, [0x1020, 0x5020, 0x5000])  # ALRS left high is dropped, SON / SFTY kept


class LineRulesTest(unittest.TestCase):
    """Duck-typed line rules (``bus.line_rules`` / ``bus.line``): no coils / no 0x45 / no broadcast."""

    def ec_on_rcon_like_line(self) -> tuple[Elecylinder, SimulatedEC, FrameRecorder]:
        ec, sim, rec = make_ec(time_scale=5.0)
        ec.bus.line_rules = SimpleNamespace(kind="rcon", coils=False, iai_commands=False, broadcast=False)
        return ec, sim, rec

    def test_register_path_and_refusals(self) -> None:
        ec, sim, rec = self.ec_on_rcon_like_line()
        with ec:
            self.assertEqual(ec.stop_path, "register")
            self.assertEqual(ec.line_name, "rcon")
            ec.servo(True)
            self.assertEqual(ec.strategy.name, "register")
            ec.move_to(140.0, speed_mm_s=20.0, wait=False)
            ec.stop()
            self.assertFalse(sim.moving)
            rec.clear()
            for call in (lambda: ec.set_baudrate(115200), ec.software_reset, lambda: ec.home(method="iai"),
                         lambda: ec.set_action_mode(True), lambda: ec.move_to_position(0, speed_pct=50),
                         lambda: ec.move_to(80.0, decel_g=0.2)):  # fmt: skip
                with self.subTest(call=call), self.assertRaises(LineRuleError):
                    call()
            self.assertFalse([f for f in rec.tx if f[1] in (0x05, 0x45, 0x48) or f[0] == 0])
            self.assertTrue(issubclass(LineRuleError, UnsafeCommandError))

    def test_iai_strategy_falls_back_on_a_line_without_0x45(self) -> None:
        ec, sim, rec = make_ec(strategy="iai")
        ec.bus.line_rules = {"iai_commands": False}
        with ec:
            ec.servo(False)
            self.assertFalse(sim.servo_on)
            self.assertEqual(ec.strategy.name, "coil")
            self.assertFalse([f for f in rec.tx if f[1] == 0x45])

    def test_rcon_status_bits_are_none(self) -> None:
        ec, sim, rec = self.ec_on_rcon_like_line()
        with ec:
            st = ec.status()
        self.assertEqual(st.line, "rcon")
        self.assertIsNone(st.nv_busy)
        self.assertIsNone(st.at_backward_end)
        self.assertIsNotNone(st.manual_mode)
        words = [0] * 22
        words[5] = R.DSS1_SV | R.DSS1_HEND | R.DSS1_PEND
        words[7] = R.DSSE_PUSH | R.DSSE_PMSS
        words[20] = 0x03FF
        rcon = replace(Status.from_registers(words), line="rcon")
        self.assertEqual(rcon.position_complete_no, 0x7F)  # 7 bits on RCON (DOC ME0413 p.5-10)
        self.assertTrue(rcon.push_contact)  # DSSE b0 undefined there: PEND + PUSH
        self.assertEqual(Status.from_registers(words).position_complete_no, 0x3FF)
        self.assertFalse(Status.from_registers(words).push_contact)

    def test_gateway_line_object(self) -> None:
        try:
            from iai_actuator_driver.gateway import LINE_RCON, Line
        except ImportError:  # pragma: no cover - the gateway module is optional for the axis layer
            self.skipTest("iai_actuator_driver.gateway not available")
        ec, sim, rec = make_ec()
        with ec:
            ec.bus.line = Line(LINE_RCON)
            self.assertEqual((ec.stop_path, ec.line_name), ("register", "rcon"))
            self.assertEqual(ec.status().line, "rcon")
            with self.assertRaises(UnsafeCommandError):
                ec.software_reset()


class SimLineFeaturesTest(unittest.TestCase):
    def test_function_codes_and_broadcast(self) -> None:
        sim = SimulatedEC(function_codes=(0x03, 0x06, 0x10))
        bus = ModbusRTU(sim, min_gap_s=0.0)
        with self.assertRaises(ModbusException) as caught:
            bus.write_coil(1, R.COIL_STOP, True)
        self.assertEqual(caught.exception.code, 0x01)
        self.assertFalse(sim.broadcast)
        self.assertTrue(SimulatedEC().broadcast)


# =========================================================================== status


class StatusFlagsTest(unittest.TestCase):
    def test_22_words_and_poss(self) -> None:
        ec, sim, rec = make_ec()
        with ec:
            st = ec.status()
            self.assertIn(hexbytes("01 03 90 00 00 16 E9 04"), rec.tx)
            self.assertEqual((st.position_complete_no, st.cold_start_alarm, st.rtc_in_use), (0, False, False))
            ec.move_to_position(2, allow_above_caps=True)
            self.assertEqual(ec.status().position_complete_no, 2)
            ec.move_to(80.0, speed_mm_s=50.0)
            self.assertEqual(ec.status().position_complete_no, 0)  # 0 after a 0x9900 move

    def test_poss_after_a_push_row_miss(self) -> None:
        ec, sim, _ = make_ec(profile="gripper", time_scale=50.0)
        with ec:
            ec.move_to_position(0)
            st = ec.move_to_position(1)
            self.assertTrue(st.push_missed)
            self.assertEqual(st.position_complete_no, 1)  # valid at a push miss too (DOC ME0162-12A p.4-24)

    def test_condition_flags(self) -> None:
        ec, sim, _ = make_ec(rtc=True)
        with ec:
            st = ec.status()
            self.assertEqual(
                (st.rtc_in_use, st.drive_power_cut, st.emg_input, st.enable_off, st.auto_servo_off, st.nv_busy,
                 st.manual_mode, st.battery_low, st.excitation_done),
                (True, False, False, False, False, False, True, False, True),
            )  # fmt: skip
            cold = next(c for c in range(0x080, 0x100) if alarm_level(c) == LEVEL_COLD_START)
            sim.trigger_alarm(cold)
            self.assertTrue(ec.status().cold_start_alarm)
            sim.set_inputs(drive_cut=True)
            st = ec.status()
            self.assertTrue(st.drive_power_cut)
            self.assertFalse(st.raw["STAT"] & R.STAT_MPOW)
            sim.set_inputs(drive_cut=False, emg=True, enable=False)
            st = ec.status()
            self.assertTrue(st.emg_input and st.emergency_stop and st.enable_off)
            data = st.to_dict()
            for key in ("position_complete_no", "cold_start_alarm", "rtc_in_use", "drive_power_cut", "emg_input",
                        "enable_off", "auto_servo_off", "nv_busy", "manual_mode", "battery_low"):  # fmt: skip
                self.assertIn(key, data)

    def test_nv_busy_while_a_commit_runs(self) -> None:
        ec, sim, _ = make_ec(parw_busy_s=5.0)
        with ec:
            ec.bus.write_coil(ec.slave, R.COIL_PARW, True)  # the commit runs for 5 s
            self.assertTrue(ec.status().nv_busy)

    def test_short_reads(self) -> None:
        for n in (8, 14, 20):
            st = Status.from_registers([0] * n)
            self.assertIsNone(st.position_complete_no)
            self.assertIsNone(st.cold_start_alarm)
        self.assertIsNone(Status.from_registers([0] * 8).auto_servo_off)
        self.assertIs(Status.from_registers([0] * 8).drive_power_cut, False)  # DSSE MPUV alone (no STAT)

    def test_fallback_22_then_20_then_8(self) -> None:
        class Refusing(SimulatedEC):
            limit = 20

            def on_request(self, frame: bytes) -> bytes | None:
                if len(frame) == 8 and frame[0] == self.slave and frame[1] == 0x03:
                    address, count = struct.unpack(">HH", frame[2:6])
                    if address == R.MONITOR_BASE and count > self.limit:
                        raise SimException(0x02)
                return NotImplemented  # type: ignore[return-value]

        sim = Refusing(time_scale=50.0)
        ec, rec = open_ec(sim)
        with ec:
            st = ec.status()
            self.assertIn("ZONS", st.raw)
            self.assertNotIn("POSS", st.raw)
            sim.limit = 8
            st = ec.status()
            self.assertNotIn("ZONS", st.raw)
            self.assertEqual(ec.status_words, R.STATUS_BLOCK_MIN_WORDS)

    def test_short_directory_record_reads_20(self) -> None:
        ec, sim, rec = make_ec(monitor_record_words=20)  # the directory's monitor record: 20 words
        with ec:
            ec.identify()  # reads the directory
            rec.clear()
            ec.status()
            self.assertEqual(rec.tx[0][:6], hexbytes("01 03 90 00 00 14"))


# =========================================================================== servo


class ServoReasonsTest(unittest.TestCase):
    def test_refused_before_sending(self) -> None:
        for inputs, words in (({"enable": False}, "enable switch"), ({"drive_cut": True}, "drive power")):
            with self.subTest(inputs=inputs):
                ec, sim, rec = make_ec()
                with ec:
                    sim.set_inputs(**inputs)  # type: ignore[arg-type]
                    self.assertFalse(sim.servo_on)
                    rec.clear()
                    with self.assertRaises(ServoBlockedError) as caught:
                        ec.servo(True)
                    self.assertIn(words, str(caught.exception))
                    self.assertTrue(any(words in r for r in caught.exception.reasons))
                    self.assertFalse([f for f in coil_writes(rec.tx) if f[2:4] == b"\x04\x03"])
                    self.assertIsInstance(caught.exception, EcNotReadyError)

    def test_emergency_stop_refused(self) -> None:
        ec, sim, _ = make_ec()
        with ec:
            sim.set_inputs(emg=True)
            with self.assertRaises(EcNotReadyError):
                ec.servo(True)

    def test_con_first_servo_on_timeout_from_no29_no30(self) -> None:
        ec, sim, _ = make_ec(profile="pcon", cold_boot=True)
        with ec:
            st = ec.status()
            self.assertFalse(st.servo_on or st.excitation_done)
            timeout, why = ec.servo_on_timeout()
            # PCON item 9.1 = 0x0002001E: No.29 = 30 ms, No.30 = 1 -> 0.006 + (0.22 + 0.030) x 10 + 0.020 + 1 s
            self.assertAlmostEqual(timeout, 3.526, places=3)
            self.assertIn("No.29 30 ms", why)
            st = ec.servo(True)  # 2.5 s simulated at time_scale 50
            self.assertTrue(st.servo_on and st.excitation_done)
            self.assertEqual(ec.servo_on_timeout()[0], 3.0)

    def test_timeout_explains_a_running_excitation_detection(self) -> None:
        ec, sim, _ = make_ec(profile="pcon", cold_boot=True, time_scale=1.0)
        with ec:
            with self.assertRaises(ServoBlockedError) as caught:
                ec.servo(True, timeout=0.05)
            self.assertIn("excitation detection", str(caught.exception))

    def test_ec_cold_boot_keeps_the_default_timeout(self) -> None:
        ec, sim, _ = make_ec(cold_boot=True)
        with ec:
            self.assertEqual(ec.servo_on_timeout()[0], 3.0)
            self.assertTrue(ec.servo(True).servo_on)


# =========================================================================== automatic servo OFF


class AutoServoOffTest(unittest.TestCase):
    def test_move_switches_the_servo_back_on_first(self) -> None:
        ec, sim, rec = make_ec(auto_servo_off_s=10.0)  # 0.2 s real at time_scale 50
        with ec:
            ec.move_to(80.0, speed_mm_s=50.0)
            self.assertTrue(wait_for(lambda: sim.auto_servo_off_active, 2.0))
            st = ec.status()
            self.assertEqual((st.servo_on, st.auto_servo_off, st.in_position), (False, True, False))
            rec.clear()
            ec.check_move(90.0, speed_mm_s=50.0)  # a check only: accepted, nothing written
            self.assertFalse([f for f in rec.tx if f[1] in (0x05, 0x06, 0x10)])
            st = ec.move_to(90.0, speed_mm_s=50.0)
            self.assertEqual(st.position_mm, 90.0)
            son = [i for i, f in enumerate(rec.tx) if f[1] == 0x05 and f[2:4] == b"\x04\x03"]
            move = [i for i, f in enumerate(rec.tx) if direct_moves([f])]
            self.assertTrue(son and move and son[0] < move[0], "SON must go out before the 0x9900 write")

    def test_wait_ends_when_the_servo_goes_off_at_the_target(self) -> None:
        ec, sim, _ = make_ec(auto_servo_off_s=0.5)  # 10 ms real: inside the 0.1 s settle window
        with ec:
            st = ec.move_to(80.0, speed_mm_s=50.0)
            self.assertEqual(st.position_mm, 80.0)

    def test_con_push_warns_when_auto_servo_off_is_enabled(self) -> None:
        ec, sim, _ = make_ec(profile="pcon")
        with ec:
            ec.identify()
            address = sim.item_address(R.CATEGORY_MOTION_PARAMS, 14)
            assert address is not None
            raw = R.from_u32_words(sim.register_value(address), sim.register_value(address + 1)) | (1 << 8)
            for offset, word in enumerate(R.to_u32_words(raw)):  # No.53 = 1
                sim.set_parameter_word(address + offset, word)
            result = ec.push(80.0, 10.0, 30.0, speed_mm_s=20.0)
            self.assertTrue(any("automatic servo OFF" in w for w in result.warnings), result.warnings)


# =========================================================================== mid-move changes


class ChangeMoveTest(unittest.TestCase):
    """The units run on a stepped clock (SteppedClockSim): a move goes on only with the requests, so it still runs
    when the change is sent however long a busy PC takes to send it (on the wall clock a move could end first)."""

    def test_change_speed_resends_the_full_block(self) -> None:
        ec, sim, rec = make_ec(sim=SteppedClockSim(step_s=0.05))
        with ec:
            ec.move_to(140.0, speed_mm_s=20.0, accel_g=0.1, wait=False)
            self.assertTrue(wait_for(lambda: ec.status().position_mm > 75.0, timeout=30.0))  # each read: 1 mm on
            self.assertIsNone(ec.change_speed(60.0))
            first, second = direct_moves(rec.tx)[-2:]
            self.assertEqual(first, (14000, 10, 2000, 10, 0, 0))
            self.assertEqual(second, (14000, 10, 6000, 10, 0, 0))  # same target, band, accel, CTLF; new VCMD
            self.assertEqual(sim.command_speed_mm_s, 60.0)
            ec.change_accel(0.2)
            self.assertEqual(direct_moves(rec.tx)[-1], (14000, 10, 6000, 20, 0, 0))
            st = ec.wait_idle(target_mm=140.0, timeout=10.0)
            self.assertEqual(st.position_mm, 140.0)

    def test_refused_without_a_move(self) -> None:
        ec, sim, rec = make_ec(sim=SteppedClockSim(step_s=0.05))
        with ec:
            with self.assertRaises(EcNotReadyError):
                ec.change_speed(30.0)
            ec.push(80.0, 10.0, 30.0, wait=False)
            with self.assertRaises(EcNotReadyError):
                ec.change_speed(5.0)
            ec.stop()
            ec.move_to(140.0, speed_mm_s=20.0, wait=False)
            with self.assertRaises(SoftLimitError):
                ec.change_speed(1000.0)  # above the cap: nothing sent
            ec.stop()

    def test_ec_smooth_accel_guard(self) -> None:
        ec, sim, rec = make_ec(sim=SteppedClockSim(step_s=0.005), max_speed_mm_s=700.0)  # 250 mm/s: 1.25 mm a read
        with ec:
            self.assertTrue(ec.is_ec)
            address = sim.item_address(R.CATEGORY_MOTION_PARAMS, R.PARAM_ITEM_CTLF)
            assert address is not None
            for offset, word in enumerate(R.to_u32_words(0x40)):  # P5 smooth accel on
                sim.set_parameter_word(address + offset, word)
            ec.move_to(140.0, speed_mm_s=250.0, accel_g=0.3, wait=False)
            with self.assertLogs("iai_actuator_driver.ec", "WARNING") as logs:
                ec.change_speed(200.0)
            self.assertTrue(any("overload" in line for line in logs.output))
            with self.assertRaisesRegex(SoftLimitError, "2 s"):
                ec.change_accel(0.01)  # 200 / (0.01 x 9800) = 2.04 s
            ec.stop()

    def test_con_s_motion_change_warns(self) -> None:
        ec, sim, rec = make_ec(sim=SteppedClockSim(step_s=0.05, profile="pcon"))
        with ec:
            ec.identify()
            ec.move_to(140.0, speed_mm_s=20.0, accel_g=0.3, wait=False, accel_pattern="s_motion")
            with self.assertLogs("iai_actuator_driver.ec", "WARNING") as logs:
                ec.change_speed(40.0)
            self.assertTrue(any("trapezoid" in line for line in logs.output))
            self.assertEqual(direct_moves(rec.tx)[-1][5], R.CTLF_MOD_S_MOTION)
            ec.stop()


# =========================================================================== cancel a paused move


class CancelPausedTest(unittest.TestCase):
    def test_coil_form(self) -> None:
        ec, sim, rec = make_ec(time_scale=5.0)
        with ec:
            ec.move_to(140.0, speed_mm_s=20.0, wait=False)
            self.assertTrue(wait_for(lambda: sim.position_mm > 75.0))
            ec.pause()
            self.assertIn(hexbytes("01 05 04 0A FF 00 AD 08"), rec.tx)
            rec.clear()
            st = ec.cancel_paused_move()
            self.assertEqual(
                coil_writes(rec.tx),
                [hexbytes("01 05 04 07 FF 00 3C CB"), hexbytes("01 05 04 07 00 00 7D 3B"),
                 hexbytes("01 05 04 0A 00 00 EC F8")],
            )  # fmt: skip
            self.assertFalse(st.paused)
            stopped = sim.position_mm
            time.sleep(0.3)
            self.assertAlmostEqual(sim.position_mm, stopped, places=2)
            self.assertLess(stopped, 140.0)

    def test_register_form(self) -> None:
        ec, sim, rec = make_ec(strategy="register", time_scale=5.0)
        with ec:
            ec.move_to(140.0, speed_mm_s=20.0, wait=False)
            ec.pause()
            rec.clear()
            ec.cancel_paused_move()
            self.assertEqual(
                drg1_writes(rec.tx),
                [CANCEL_PAUSED, PAUSE,
                 SERVO_ON],
            )  # fmt: skip
            time.sleep(0.2)
            self.assertFalse(sim.moving)

    def test_refusals(self) -> None:
        ec, sim, rec = make_ec(time_scale=5.0)
        with ec:
            with self.assertRaises(EcNotReadyError):
                ec.cancel_paused_move()
            ec.move_to(140.0, speed_mm_s=20.0, wait=False)
            ec.pause()
            sim.trigger_alarm(0x0E0)
            rec.clear()
            with self.assertRaises(EcAlarmError):
                ec.cancel_paused_move()
            self.assertEqual(coil_writes(rec.tx), [])


# =========================================================================== teaching


class TeachTest(unittest.TestCase):
    def test_table_path(self) -> None:
        ec, sim, rec = make_ec(max_speed_mm_s=700.0)  # row 0 runs at 108.50 mm/s
        with ec:
            ec.move_to(60.0, speed_mm_s=50.0)
            entry = ec.teach_position(0)  # allow_nonvolatile: no longer needed
            self.assertEqual((entry.position_mm, entry.checksum_ok), (60.0, True))
            self.assertEqual(sim.nonvolatile_writes, 1)
            self.assertEqual(ec.read_position_table(0).speed_mm_s, 108.5)  # the other fields are kept
            sim.homed = False
            with self.assertRaisesRegex(EcNotReadyError, "not homed"):
                ec.teach_position(0, allow_nonvolatile=True)

    def test_coil_path_frames(self) -> None:
        ec, sim, rec = make_ec(profile="pcon")
        with ec:
            rec.clear()
            entry = ec.teach_position(5, allow_nonvolatile=True, method="coil")
            self.assertEqual(entry.position_mm, 60.0)
            frames = [f for f in rec.tx if f[1] in (0x05, 0x06)]
            self.assertEqual(
                frames,
                [hexbytes("01 05 04 14 FF 00 CD 0E"), hexbytes("01 06 0D 03 00 05 BB 65"),
                 hexbytes("01 05 04 15 FF 00 9C CE"), hexbytes("01 05 04 15 00 00 DD 3E"),
                 hexbytes("01 05 04 14 00 00 8C FE")],
            )  # fmt: skip


# =========================================================================== acceleration pattern


class AccelPatternTest(unittest.TestCase):
    def test_ctlf_bits(self) -> None:
        ec, sim, rec = make_ec(profile="pcon")
        with ec:
            for pattern, bits in (("s_motion", 0x0040), ("filter", 0x0080), ("trapezoid", 0), (None, 0)):
                with self.subTest(pattern=pattern):
                    ec.move_to(80.0 if pattern != "filter" else 100.0, speed_mm_s=50.0, accel_g=0.3,
                               accel_pattern=pattern)  # fmt: skip
                    self.assertEqual(direct_moves(rec.tx)[-1][5], bits)
                    self.assertEqual(sim.accel_pattern, pattern or "trapezoid")
                    ec.move_to(60.0, speed_mm_s=50.0)

    def test_refusals(self) -> None:
        ec, sim, rec = make_ec(max_speed_mm_s=700.0)
        with ec:
            slow_s_motion = {"accel_pattern": "s_motion", "speed_mm_s": 250.0, "accel_g": 0.01}  # 2.55 s accel
            for kwargs in ({"accel_pattern": "jerk"}, slow_s_motion):
                with self.subTest(kwargs=kwargs), self.assertRaises(SoftLimitError):
                    ec.move_to(100.0, **kwargs)  # type: ignore[arg-type]
            ec.axis_limits = replace(ec.read_axis_limits(), move_patterns=0x0001)  # MVPT: trapezoid only
            with self.assertRaisesRegex(SoftLimitError, "MVPT"):
                ec.move_to(100.0, accel_pattern="filter")
            self.assertEqual(direct_moves(rec.tx), [])

    def test_ec_pattern_runs(self) -> None:  # VERIFIED on the EC-GRBP8M gripper: s_motion and filter run
        ec, sim, rec = make_ec()
        with ec:
            self.assertTrue(ec.is_ec)
            st = ec.move_to(100.0, speed_mm_s=50.0, accel_pattern="s_motion")
            self.assertAlmostEqual(st.position_mm, 100.0, delta=0.1)


# =========================================================================== push speed, minimum speed


class PushSpeedTest(unittest.TestCase):
    def test_push_speed_from_the_unit(self) -> None:
        expected = {"slider": 30.0, "rod": 20.0, "gripper": 5.0, "pcon": 20.0}
        for profile, speed in expected.items():
            with self.subTest(profile=profile):
                ec, sim, _ = make_ec(profile=profile)
                with ec:
                    self.assertEqual(ec.push_speed_mm_s(), (speed, "PSHV 8.15"))
                    self.assertEqual(ec.push_speed_mm_s(3.0), (3.0, "the commanded speed"))
                    self.assertEqual(ec.limits()["push"]["speed"], speed)

    def test_gripper_push_runs_at_5_mm_s_and_the_timeout_covers_it(self) -> None:
        ec, sim, rec = make_ec(profile="gripper", time_scale=20.0)
        with ec:
            ec.move_to(3.0, speed_mm_s=20.0)
            started = time.perf_counter()
            result = ec.push(4.0, 5.0, 60.0, speed_mm_s=20.0)  # 5 mm at 5 mm/s = 1 s simulated
            self.assertTrue(result.missed)
            self.assertEqual(sim.push_speed_mm_s, 5.0)
            self.assertGreaterEqual((time.perf_counter() - started) * 20.0, 1.0)

    def test_fallback_without_the_items(self) -> None:
        ec, sim, _ = make_ec(missing_items=[(R.CATEGORY_MOTION_PARAMS, R.PARAM_ITEM_PSHV)])  # no PSHV 8.15
        with ec:
            ec.axis_limits = replace(ec.read_axis_limits(), max_push_speed_mm_s=None)
            self.assertEqual(ec.push_speed_mm_s()[0], R.PUSH_SPEED_MAX_MM_S)

    def test_recommended_minimum_speed(self) -> None:
        cases = {"rod": 25.0, "gripper": 5.0}
        for profile, minimum in cases.items():
            with self.subTest(profile=profile):
                ec, sim, _ = make_ec(profile=profile)
                with ec:
                    self.assertEqual(ec.limits()["speed"]["recommended_min"], minimum)
        ec, sim, _ = make_ec(profile="rod")
        with ec:
            with self.assertLogs("iai_actuator_driver.ec", "WARNING") as logs:
                st = ec.move_to(20.0, speed_mm_s=10.0)  # below 25 mm/s: a warning, never a refusal
            self.assertEqual(st.position_mm, 20.0)
            self.assertTrue(any("documented minimum 25" in line for line in logs.output))
        self.assertEqual(R.recommended_min_speed("EC-DS4H-150-1", 10.0)[0], 30.0)
        self.assertEqual(R.recommended_min_speed("EC-RTC12-1", None)[0], 20.0)
        self.assertEqual(R.recommended_min_speed(None, 16.0)[0], 20.0)  # lead x 1.25 (DOC ME0155 p.139)
        self.assertEqual(R.recommended_min_speed(None, None)[0], None)


if __name__ == "__main__":
    unittest.main()
