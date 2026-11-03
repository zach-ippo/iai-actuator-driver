"""Driver behaviour based on observations on the EC-DS4H slider and the EC-GRBP8M gripper.

1. Position markers from ZONS 0x9013 b12/b13/b14 (DOPM differs per model).
2. STOP stops a Modbus-commanded move with PMSS = 0 (VERIFIED on the slider); PMSL only switched at rest
   (PMSL ON mid-move stops a move).
3. Baud-rate auto-detection in place (Elecylinder.open, CLI).
4. Unit codes: 0x0A is a gripper; display name from the 0x0600 model string.
5. Speed cap: min(max_speed_mm_s, the EC's maximum speed MAXV item 7.4; 0x0812 only as the fallback).
Plus the 2-position gripper simulator profile all of the above is exercised against.
"""

from __future__ import annotations

import contextlib
import io
import json
import tempfile
import time
import unittest
from collections.abc import Callable
from pathlib import Path
from unittest import mock

from ec_testutil import FakeClock, FrameRecorder

from iai_actuator_driver import cli
from iai_actuator_driver import modbus as modbus_module
from iai_actuator_driver import registers as R
from iai_actuator_driver.check import PASS, run_check
from iai_actuator_driver.ec import Elecylinder, PositionEntry, Status
from iai_actuator_driver.errors import EcNotReadyError, ModbusTimeout, SoftLimitError
from iai_actuator_driver.modbus import ModbusRTU
from iai_actuator_driver.sim import PROFILES, SimulatedEC


def words(text: str) -> list[int]:
    return [int(w, 16) for w in text.split()]


# Monitor words 0x9000-0x9013 read live on the EC-DS4H slider and the EC-GRBP8M gripper.
SLIDER_LIVE = words("0000 1D4B 0000 0000 000C 3099 8000 33C2 0008 801F 0000 0000 0000 0283 FFFF FFFF 0022 20C4 0300 4003")
GRIPPER_LIVE = words("0000 03B6 0000 0000 0006 3098 8002 32C2 0008 801F 0000 0000 0000 0000 0000 0000 0000 AEE9 0300 2003")


def open_ec(sim: SimulatedEC, **kwargs: object) -> tuple[Elecylinder, FrameRecorder]:
    """Elecylinder on ``sim`` (stroke from the EC unless given), with a frame recorder."""
    rec = FrameRecorder()
    kwargs.setdefault("poll_interval_s", 0.005)
    ec = Elecylinder(ModbusRTU(sim, frame_logger=rec, min_gap_s=0.0), owns_bus=True, **kwargs)  # type: ignore[arg-type]
    return ec, rec


def coil_frames(rec: FrameRecorder, coil: int) -> list[tuple[int, bool]]:
    """(index in rec.tx, on) for every FC05 write to ``coil``."""
    return [
        (i, f[4] == 0xFF) for i, f in enumerate(rec.tx) if f[1] == 0x05 and int.from_bytes(f[2:4], "big") == coil
    ]


def raw_move(bus: ModbusRTU, target_mm: float, speed_mm_s: float) -> None:
    """A move sent by another master (e.g. a second tool), bypassing the driver's checks."""
    bus.write_registers(1, R.DIRECT_MOVE_BASE, [*R.to_i32_words(round(target_mm * 100)), 0, 10,
                                                *R.to_u32_words(round(speed_mm_s * 100)), 30, 0, 0])


# =========================================================================== 1. position markers


class PositionMarkerTest(unittest.TestCase):
    def test_slider_live_words(self) -> None:
        st = Status.from_registers(SLIDER_LIVE)  # 74.99 mm, stored middle 75.00
        self.assertEqual((st.at_backward_end, st.at_middle, st.at_forward_end), (False, True, False))
        self.assertEqual(st.raw["DOPM"], 0x000C)
        self.assertEqual(st.raw["ZONS"], 0x4003)

    def test_gripper_live_words_dopm_b2_is_not_middle(self) -> None:
        st = Status.from_registers(GRIPPER_LIVE)  # 9.50 mm, 2-position gripper
        self.assertEqual(st.position_mm, 9.5)
        self.assertTrue(st.raw["DOPM"] & 0x0004, "DOPM b2 (*ALM on the gripper) is set in the live word")
        self.assertIs(st.at_middle, False)  # DOPM b2 is *ALM here, not the Middle marker
        self.assertIs(st.at_forward_end, True)  # ZONS b13
        self.assertIs(st.at_backward_end, False)
        self.assertFalse(st.alarm)

    def test_no_marker_without_zons(self) -> None:
        for n in (8, 14):
            st = Status.from_registers(GRIPPER_LIVE[:n])
            self.assertEqual((st.at_backward_end, st.at_middle, st.at_forward_end), (None, None, None), n)
            self.assertIsNone(st.to_dict()["at_middle"])
            self.assertNotIn("ZONS", st.raw)

    def test_driver_reads_twenty_two_words(self) -> None:
        ec, rec = open_ec(SimulatedEC())
        with ec:
            st = ec.status()
        # 0x9000-0x9015 (PNOW..SSSE): 01 03 90 00 00 16 E9 04 (DOC ME0162-12A p.4-7, 5-10)
        self.assertIn(bytes.fromhex("01 03 90 00 00 16 E9 04".replace(" ", "")), rec.tx)
        for name in ("DOPM", "ZONS", "STIM", "SIPM", "DEVI", "POSS", "SSSE"):
            self.assertIn(name, st.raw)

    def test_gripper_sim_matches_the_live_profile(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=20.0)
        ec, _ = open_ec(sim)
        with ec:
            st = ec.status()
            self.assertEqual((st.raw["DOPM"], st.raw["ZONS"]), (0x0006, 0x2003))  # as read live at 9.50 mm
            self.assertIs(st.at_forward_end, True)
            self.assertIs(st.at_middle, False)
            st = ec.move_to(3.0, speed_mm_s=20.0)  # row 0, "open"
            self.assertEqual((st.raw["DOPM"], st.raw["ZONS"]), (0x0005, 0x1003))  # LS0 + *ALM (b2) / ZONS b12
            self.assertEqual((st.at_backward_end, st.at_middle, st.at_forward_end), (True, False, False))
            # Row 2 (2.50 mm) lights LS2 on the 2-position gripper too: ZONS 0x4003 at 2.52 mm (VERIFIED).
            # DOPM has no LS2 bit in the 2-position layout (b2 = *ALM).
            st = ec.move_to(2.5, speed_mm_s=20.0)
            self.assertEqual((st.raw["DOPM"], st.raw["ZONS"]), (0x0004, 0x4003))
            self.assertEqual((st.at_backward_end, st.at_middle, st.at_forward_end), (False, True, False))
            # Row 1 is a push row (4.10 mm + 5.40 mm): its marker is the push end 9.50 mm, not the push zone.
            # Live: ZONS 0x0003 at 5.99 mm and at 9.00 mm.
            for target in (6.0, 9.0):
                st = ec.move_to(target, speed_mm_s=20.0)
                self.assertEqual((st.raw["DOPM"], st.raw["ZONS"]), (0x0004, 0x0003), target)
                self.assertIs(st.at_forward_end, False, target)
            sim.trigger_alarm(0x0E0)
            self.assertEqual(ec.status().raw["DOPM"] & R.DOPM_2POS_NO_ALARM, 0)  # *ALM drops on an alarm

    def test_markers_follow_position_while_moving(self) -> None:
        """Gripper 1.00 -> 6.00 mm at 10 mm/s: live ZONS 0x1003 at 2.91 mm while moving."""
        clock = FakeClock()
        sim = SimulatedEC(profile="gripper", clock=clock, position_mm=1.0)
        ec, _ = open_ec(sim)
        with ec:
            raw_move(ec.bus, 6.0, 10.0)
            seen = {}
            while sim.moving:
                clock.advance(0.001)
                st = ec.status()
                seen[round(st.position_mm, 2)] = st
                markers = (st.at_backward_end, st.at_middle, st.at_forward_end)
                pos = round(st.position_mm * 100)
                self.assertEqual(markers, (abs(pos - 300) <= 10, abs(pos - 250) <= 10, False), st.position_mm)
            st = seen[2.91]
            self.assertTrue(st.moving)
            self.assertEqual(st.raw["ZONS"], 0x1003)
            self.assertEqual(st.raw["DOPM"], 0x0005)  # live 0x0005 too
            self.assertTrue(seen[2.5].moving)
            self.assertEqual(seen[2.5].raw["ZONS"], 0x4003)  # live: 0x4003 at 2.52 mm while moving
            self.assertEqual(seen[3.11].raw["ZONS"], 0x0003)
            self.assertEqual(ec.status().raw["ZONS"], 0x0003)  # arrived at 6.00 mm: no marker

    def test_push_row_marker_only_at_the_push_end(self) -> None:
        """Gripper grip push, live: ZONS 0x0003 along the whole push 4.44 -> 9.40 mm, 0x2003 at 9.50 mm."""
        clock = FakeClock()
        sim = SimulatedEC(profile="gripper", clock=clock, position_mm=3.0)
        bus = ModbusRTU(sim, min_gap_s=0.0)
        bus.write_register(1, R.COMMAND1_BASE, 1)  # stored row 1: grip push
        while sim.moving:
            clock.advance(0.002)
            pos, zons = sim.position_mm, bus.read_registers(1, R.ZONS, 1)[0]
            if 4.1 <= pos <= 9.39:
                self.assertEqual(zons & 0x7000, 0, pos)
        self.assertEqual(sim.position_mm, 9.5)
        self.assertEqual(bus.read_registers(1, R.ZONS, 1)[0], 0x2003)
        self.assertEqual(bus.read_registers(1, R.DOPM, 1)[0], 0x0004)  # live after the missed grip (PSFL)


class GripperPositionWriteTest(unittest.TestCase):
    def test_ec_own_unknown_flag_bit_is_kept(self) -> None:
        """The gripper's rows carry CTLF 0x0400 (VERIFIED); rewriting row 0 with its own flags gives no warning, another
        unknown bit is written with one."""
        import dataclasses

        ec = Elecylinder.open(SimulatedEC(profile="gripper", time_scale=20.0))
        try:
            row0 = ec.read_position_table(0)
            self.assertEqual(row0.control_flags & 0x0400, 0x0400)
            with self.assertNoLogs("iai_actuator_driver", "WARNING"):
                written = ec.write_position_table(0, dataclasses.replace(row0, position_mm=3.5, checksum=None))
            self.assertAlmostEqual(written.position_mm, 3.5)
            self.assertTrue(ec.read_position_table(0).checksum_ok)
            changed = dataclasses.replace(row0, control_flags=row0.control_flags | 0x0800, checksum=None)
            with self.assertLogs("iai_actuator_driver", "WARNING") as logs:
                written = ec.write_position_table(0, changed)
            self.assertIn("unknown bits", logs.output[0])
            self.assertEqual(written.control_flags, row0.control_flags | 0x0800)
        finally:
            ec.close()


# =========================================================================== 2. STOP / PMSL


class ModbusControlTest(unittest.TestCase):
    def test_sim_runs_moves_and_stop_with_pmss_0(self) -> None:
        """As on the EC-DS4H slider: a 0x9900 move runs and STOP stops it while PMSS = 0."""
        clock = FakeClock()
        sim = SimulatedEC(clock=clock, position_mm=20.0, pmsl=False)
        bus = ModbusRTU(sim, min_gap_s=0.0)
        raw_move(bus, 60.0, 10.0)
        clock.advance(1.0)
        self.assertTrue(sim.moving)
        bus.write_coil(1, R.COIL_STOP, True)
        clock.advance(2.0)
        self.assertFalse(sim.moving)
        self.assertLess(sim.position_mm, 35.0)
        dss1 = bus.read_registers(1, R.DSS1, 1)[0]
        self.assertTrue(dss1 & R.DSS1_PEND)
        self.assertFalse(dss1 & R.DSS1_TARGET_REACHED)
        self.assertFalse(sim.pmsl)

    def test_sim_pmsl_on_mid_move_stops_the_move(self) -> None:
        clock = FakeClock()
        sim = SimulatedEC(clock=clock, position_mm=0.0, pmsl=False)
        bus = ModbusRTU(sim, min_gap_s=0.0)
        raw_move(bus, 20.0, 10.0)
        clock.advance(1.0)
        bus.write_coil(1, R.COIL_PMSL, True)
        self.assertIn("move stopped by PMSL", sim.events)
        clock.advance(2.0)
        self.assertFalse(sim.moving)
        self.assertLess(sim.position_mm, 15.0)
        dss1, _, dsse = bus.read_registers(1, R.DSS1, 3)
        self.assertTrue(dss1 & R.DSS1_PEND)
        self.assertFalse(dss1 & R.DSS1_TARGET_REACHED)  # live: PEND = 1, b0 = 0
        self.assertTrue(dsse & R.DSSE_PMSS)
        raw_move(bus, 20.0, 10.0)  # live: the re-sent move then reached 20.00 with b0 = 1
        clock.advance(5.0)
        self.assertEqual(sim.position_mm, 20.0)
        self.assertTrue(bus.read_registers(1, R.DSS1, 1)[0] & R.DSS1_TARGET_REACHED)

    def test_driver_stop_works_with_pmss_0_and_leaves_pmsl_alone(self) -> None:
        sim = SimulatedEC(time_scale=5.0, pmsl=False)
        ec, rec = open_ec(sim)
        with ec:
            raw_move(ec.bus, 140.0, 20.0)
            st = ec.stop(wait=True)  # must not raise or claim STOP is ignored
            self.assertIsNotNone(st)
            self.assertFalse(st.moving)
            self.assertFalse(st.modbus_enabled)
            self.assertLess(sim.position_mm, 100.0)
            self.assertFalse(sim.moving)
            self.assertFalse(sim.coil_state(R.COIL_STOP), "STOP held after the axis stopped")
            self.assertEqual(coil_frames(rec, R.COIL_PMSL), [], "PMSL written by stop()")
            self.assertFalse(sim.pmsl)
            self.assertEqual([on for _, on in coil_frames(rec, R.COIL_STOP)], [True, False])

    def test_pmsl_is_never_switched_while_moving(self) -> None:
        sim = SimulatedEC(time_scale=2.0, pmsl=False)
        ec, rec = open_ec(sim)
        with ec:
            raw_move(ec.bus, 140.0, 20.0)  # e.g. a PLC or another tool moving the axis
            rec.clear()
            with self.assertRaisesRegex(EcNotReadyError, "moving"):
                ec.enable_modbus_control()
            with self.assertRaisesRegex(EcNotReadyError, "moving"):
                ec.move_to(100.0, speed_mm_s=20.0)
            with self.assertRaisesRegex(EcNotReadyError, "moving"):
                ec.set_safety_speed(True)
            with self.assertRaisesRegex(EcNotReadyError, "moving"):
                ec.disable_modbus_control()
            self.assertEqual({f[1] for f in rec.tx}, {0x03}, "only reads may be sent while PIO moves the axis")
            self.assertTrue(sim.moving, "the move was disturbed")
            ec.stop(wait=True)
            self.assertTrue(ec.enable_modbus_control().modbus_enabled)  # at rest: allowed

    def test_control_taken_at_rest_before_the_first_move(self) -> None:
        sim = SimulatedEC(time_scale=20.0, pmsl=False)
        ec, rec = open_ec(sim)
        with ec:
            st = ec.move_to(80.0, speed_mm_s=50.0)
        self.assertEqual(st.position_mm, 80.0)
        self.assertTrue(sim.pmsl)
        pmsl = coil_frames(rec, R.COIL_PMSL)
        moves = [i for i, f in enumerate(rec.tx) if f[1] == 0x10]
        self.assertEqual([on for _, on in pmsl], [True])
        self.assertLess(pmsl[0][0], moves[0], "PMSL must be written before the move, at rest")

    def test_servo_on_takes_control_first(self) -> None:
        sim = SimulatedEC(pmsl=False)
        sim.servo_on = False
        ec, rec = open_ec(sim)
        with ec:
            ec.servo(True)
        self.assertTrue(sim.servo_on and sim.pmsl)
        pmsl, son = coil_frames(rec, R.COIL_PMSL), coil_frames(rec, R.COIL_SON)
        self.assertLess(pmsl[0][0], son[0][0])

    def test_connect(self) -> None:
        sim = SimulatedEC(pmsl=False)
        ec, rec = open_ec(sim)
        with ec:
            info = ec.connect(take_control=False)
            self.assertLessEqual({f[1] for f in rec.tx}, {0x03, 0x11}, "connect(take_control=False) wrote something")
            self.assertEqual(info.max_speed_mm_s, 700.0)
            self.assertFalse(sim.pmsl)
            ec.connect()
            self.assertTrue(sim.pmsl)

    def test_pcon_pc_control_is_taken_back_while_paused_at_rest_in_pio_mode(self) -> None:
        """The PCON with nothing on its PIO connector: in PIO mode the servo goes off and DSS1 STP (the unwired *STP
        input) shows at rest (VERIFIED on the PCON-CB, 2026-11-02). Taking PC control back is not refused for that
        pause, by every way the driver takes it."""
        sim = SimulatedEC(profile="pcon", time_scale=20.0)
        ec, rec = open_ec(sim)
        with ec:
            ec.set_action_mode(False)
            st = ec.status()
            self.assertEqual((st.modbus_enabled, st.paused, st.servo_on, st.moving, st.idle), (False, True, False, False, False))
            st = ec.set_action_mode(True)
            self.assertEqual((st.modbus_enabled, st.paused, st.servo_on), (True, False, False))  # the servo stays off
            ec.disable_modbus_control()
            self.assertTrue(ec.status().paused)
            self.assertTrue(ec.enable_modbus_control().modbus_enabled)
            ec.disable_modbus_control()
            ec.connect()  # the session start takes it too
            self.assertTrue(sim.pmsl)
            ec.disable_modbus_control()
            rec.clear()
            ec.servo(True)  # a command takes control first (auto_modbus_control), then the servo comes on
            st = ec.status()
            self.assertEqual((st.modbus_enabled, st.servo_on, st.idle), (True, True, True))
            pmsl, son = coil_frames(rec, R.COIL_PMSL), coil_frames(rec, R.COIL_SON)
            self.assertLess(pmsl[0][0], son[0][0])

    def test_pcon_servo_comes_on_after_taking_pc_control_with_the_action_mode(self) -> None:
        """On the PCON-CB every 0x45 cmd 0x0C with op1 = 1 cleared DSSE bit 12, and servo ON was then ignored (no
        alarm, no cause shown) until a PMSL coil OFF (VERIFIED, 2026-11-02; the simulator's PCON models it).
        set_action_mode(PC) takes control with the PMSL coil and changes the safety speed with the SFTY coil."""
        sim = SimulatedEC(profile="pcon", time_scale=20.0)
        ec, rec = open_ec(sim)
        with ec:
            ec.set_action_mode(False)  # PIO with nothing on the PIO connector: paused, servo off
            self.assertFalse(ec.status().servo_on)
            rec.clear()
            self.assertTrue(ec.set_action_mode(True).modbus_enabled)
            self.assertTrue(ec.servo(True).servo_on)  # before the fix: "servo did not turn on within 3 s"
            self.assertEqual([f for f in rec.tx if f[1] == 0x45 and f[3] == R.IAI_CMD_ACTION_MODE and f[5] == 1], [])
            self.assertEqual([on for _, on in coil_frames(rec, R.COIL_PMSL)], [True])
            ec.servo(False)
            rec.clear()
            self.assertTrue(ec.set_action_mode(True, True).safety_speed)  # already in PC: nothing for control
            self.assertEqual(coil_frames(rec, R.COIL_PMSL), [])
            self.assertEqual([on for _, on in coil_frames(rec, R.COIL_SFTY)], [True])
            self.assertFalse(ec.set_action_mode(True, False).safety_speed)
            self.assertTrue(ec.servo(True).servo_on)

    def test_pc_control_is_still_refused_while_a_paused_axis_moves(self) -> None:
        """PMSS 0 and STP set: the pause alone is no refusal when the servo is off, but the servo on (a PLC holding
        *STP), a MOVE bit, a home return, a push or a VNOW above 0 still is (nothing written), and handing control to
        PIO while paused at a standstill stays refused."""
        off = R.DSS1_PWR | R.DSS1_HEND | R.DSS1_STP
        servo = off | R.DSS1_SV
        cases = {"standstill, servo off": (off, R.DSSE_PSNS, 0, False),
                 "standstill, servo on": (servo, R.DSSE_PSNS, 0, True),
                 "MOVE": (off, R.DSSE_PSNS | R.DSSE_MOVE, 0, True),
                 "homing": (off, R.DSSE_PSNS | R.DSSE_GHMS, 0, True),
                 "VNOW": (off, R.DSSE_PSNS, 300, True),
                 "push": (off, R.DSSE_PSNS | R.DSSE_PUSH, 0, True),
                 "push, servo on": (servo, R.DSSE_PSNS | R.DSSE_PUSH, 0, True)}
        for name, (dss1, dsse, vnow, refused) in cases.items():
            with self.subTest(name):
                sim = SimulatedEC(pmsl=False)
                ec, rec = open_ec(sim)
                with ec:
                    for address, value in ((R.DSS1, dss1), (R.DSSE, dsse), (R.VNOW, 0), (R.VNOW + 1, vnow)):
                        sim.set_word_override(address, value)
                    self.assertTrue(ec.status().paused)
                    rec.clear()
                    if refused:
                        with self.assertRaisesRegex(EcNotReadyError, "moving"):
                            ec.enable_modbus_control()
                        with self.assertRaisesRegex(EcNotReadyError, "only switched at rest"):
                            ec.set_action_mode(True)
                        with self.assertRaises(EcNotReadyError):
                            ec.servo(True)  # the automatic take-over looks at the same rule
                        self.assertEqual({f[1] for f in rec.tx}, {0x03}, "only reads may be sent")
                        continue
                    ec.enable_modbus_control(verify=False)
                    self.assertEqual([on for _, on in coil_frames(rec, R.COIL_PMSL)], [True])
                    # in PC mode, paused at a standstill: handing control to PIO keeps the old rule (idle)
                    sim.set_word_override(R.DSSE, dsse | R.DSSE_PMSS)
                    rec.clear()
                    with self.assertRaises(EcNotReadyError):
                        ec.disable_modbus_control()
                    with self.assertRaises(EcNotReadyError):
                        ec.set_action_mode(False)
                    self.assertEqual({f[1] for f in rec.tx}, {0x03}, "only reads may be sent")

    def test_connect_while_moving_does_not_take_control(self) -> None:
        sim = SimulatedEC(time_scale=2.0, pmsl=False)
        ec, rec = open_ec(sim)
        with ec:
            raw_move(ec.bus, 140.0, 20.0)
            with self.assertLogs("iai_actuator_driver.ec", "WARNING") as logs:
                ec.connect()
            self.assertIn("not taken", logs.output[0])
            self.assertEqual(coil_frames(rec, R.COIL_PMSL), [])
            self.assertTrue(sim.moving)
            ec.stop()


# =========================================================================== 3. auto-baud


def baud_port(port: str = "COM99", baudrate: int = 115200, timeout: float = 0.2, *,
              answer_at: int = 38400) -> SimulatedEC:
    """A pretend serial port: a simulated EC on a modelled line that only gets through at ``answer_at`` baud."""
    return SimulatedEC(baudrate=baudrate, ec_baudrate=answer_at, port_name=port)


class PortFactory:
    """Stands in for ``modbus.SerialTransport``: records every port it opens (no serial port is touched)."""

    def __init__(self, answer_at: int = 38400) -> None:
        self.answer_at = answer_at
        self.ports: list[SimulatedEC] = []

    def __call__(self, port: str, baudrate: int = 115200, timeout: float = 0.2) -> SimulatedEC:
        p = baud_port(port, baudrate, timeout, answer_at=self.answer_at)
        self.ports.append(p)
        return p


class AutoBaudTest(unittest.TestCase):
    def test_open_finds_the_boot_rate_in_place(self) -> None:
        port = baud_port(answer_at=38400)
        with Elecylinder.open(port) as ec:
            self.assertEqual(ec.baudrate, 38400)
            self.assertTrue(ec.baud_detected)
            self.assertEqual(ec.status().position_mm, 73.95)
            self.assertFalse(port.closed)
        self.assertEqual(port.baud_history[1:], [38400])  # 115200 first, then 38400; never reopened
        self.assertTrue(port.closed)

    def test_open_port_name_opens_once(self) -> None:
        factory = PortFactory(answer_at=38400)
        with mock.patch.object(modbus_module, "SerialTransport", factory):
            with Elecylinder.open("COM99", timeout=0.05) as ec:
                self.assertEqual(ec.baudrate, 38400)
        self.assertEqual(len(factory.ports), 1, "the port was reopened")
        self.assertEqual(factory.ports[0].baud_history[0], 115200)

    def test_given_rate_is_used_as_is(self) -> None:
        factory = PortFactory(answer_at=38400)
        with mock.patch.object(modbus_module, "SerialTransport", factory):
            with Elecylinder.open("COM99", baudrate=38400) as ec:
                self.assertFalse(ec.baud_detected)
                ec.status()
            with self.assertRaises(ModbusTimeout):
                Elecylinder.open("COM99", baudrate=9600, timeout=0.05).status()
        self.assertEqual([p.baud_history[1:] for p in factory.ports], [[], []])

    def test_given_rate_tried_first_with_auto_baud(self) -> None:
        factory = PortFactory(answer_at=38400)
        with mock.patch.object(modbus_module, "SerialTransport", factory):
            with Elecylinder.open("COM99", baudrate=9600, auto_baud=True) as ec:
                self.assertEqual(ec.baudrate, 38400)
        self.assertEqual(factory.ports[0].baud_history[0], 9600)
        self.assertEqual(factory.ports[0].baud_history[1:], [115200, 38400])

    def test_no_answer_closes_the_port(self) -> None:
        port = baud_port(answer_at=1)
        with self.assertRaises(ModbusTimeout) as caught:
            Elecylinder.open(port)
        self.assertTrue(port.closed)
        self.assertEqual(port.baud_history[1:], [38400, 9600, 230400, 57600, 19200, 115200])  # start rate restored
        self.assertTrue(any("IA-OS" in note for note in caught.exception.__notes__))

    def test_simulator_has_no_baud(self) -> None:
        with Elecylinder.open(SimulatedEC()) as ec:
            self.assertIsNone(ec.baudrate)
            self.assertFalse(ec.baud_detected)


def run_cli(*argv: str, factory: PortFactory | None = None,
            entry: Callable[..., int] = cli.main) -> tuple[int, str, str]:
    out, err = io.StringIO(), io.StringIO()
    with mock.patch.object(modbus_module, "SerialTransport", factory or PortFactory()), \
            contextlib.redirect_stderr(err):
        code = entry(list(argv), out=out)
    return code, out.getvalue(), err.getvalue()


class CliAutoBaudTest(unittest.TestCase):
    def test_auto_detect_without_baud(self) -> None:
        factory = PortFactory(answer_at=38400)
        code, out, err = run_cli("--port", "COM99", "--timeout", "0.05", "status", factory=factory)
        self.assertEqual(code, 0, err)
        self.assertIn("73.95 mm", out)
        self.assertIn("COM99 at 38400 baud (auto-detected)", err)
        self.assertEqual(len(factory.ports), 1)
        self.assertEqual(factory.ports[0].baud_history[1:], [38400])

    def test_fixed_baud_is_not_changed(self) -> None:
        factory = PortFactory(answer_at=38400)
        code, _, err = run_cli("--port", "COM99", "--baud", "9600", "--timeout", "0.05", "status", factory=factory)
        self.assertEqual(code, 2)
        self.assertIn("ModbusTimeout", err)
        self.assertEqual(factory.ports[0].baud_history[1:], [])
        code, out, err = run_cli("--port", "COM99", "--baud", "38400", "identify")
        self.assertEqual(code, 0, err)
        self.assertIn("baud       38400", out)
        self.assertIn("fixed by --baud", err)

    def test_no_answer_at_any_rate(self) -> None:
        factory = PortFactory(answer_at=1)
        code, _, err = run_cli("--port", "COM99", "--timeout", "0.02", "status", factory=factory)
        self.assertEqual(code, 2)
        self.assertIn("did not answer at any of 115200, 38400, 9600, 230400, 57600, 19200 baud", err)
        self.assertTrue(factory.ports[0].closed)

    def test_ec_check_records_the_baud(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            report = Path(tmp) / "r.json"
            code, out, _ = run_cli("--port", "COM99", "--timeout", "0.05", "--report", str(report),
                                   entry=cli.check_main)
            self.assertEqual(code, 0, out)
            data = json.loads(report.read_text(encoding="utf-8"))
        self.assertEqual(data["baud"], 38400)
        self.assertEqual(data["port"], "COM99")


# =========================================================================== 4. unit label / model name


class UnitLabelTest(unittest.TestCase):
    def test_gripper_identity(self) -> None:
        ec, _ = open_ec(SimulatedEC(profile="gripper"))
        with ec:
            info = ec.identify()
        self.assertEqual(info.model, "EC-GRBP8M-20-1-1-PN-WA-WL2")
        self.assertEqual(info.user_name, "EC-GRBP8M-20")  # 0x061A: the user-editable unit name (set in IA-OS)
        self.assertEqual(info.short_model, "EC-GRBP8M-20")  # the compatibility alias of user_name
        # unit 0x0A is the EC(MS) "2-size EC" family; "gripper" is the actuator kind from AKID (item 7.33 = 4)
        self.assertEqual(info.controller, 'EC 24 V, unit 0x0A (EC(MS) "2-size EC")')
        self.assertEqual((info.unit_code, info.unit_name, info.firmware), (0x0A, 'EC(MS) "2-size EC"', "0002"))
        self.assertEqual((info.actuator_kind, info.actuator), (4, "gripper (AKID 4)"))
        self.assertEqual((info.max_accel_g, info.max_decel_g), (0.3, 0.3))  # MAXA / MAXD on the gripper
        self.assertEqual(len(info.categories), 25)
        self.assertEqual(info.soft_limits_mm, (-0.3, 9.8))
        self.assertEqual(info.max_speed_mm_s, 53.0)
        self.assertEqual(info.errors, ())

    def test_unknown_unit_code_is_shown_as_hex(self) -> None:
        ec, _ = open_ec(SimulatedEC(unit_code=0x0C))  # an unknown unit code (e.g. a later model)
        with ec:
            info = ec.identify()
        self.assertIsNone(info.unit_name)
        self.assertFalse(info.unit_known)
        self.assertEqual(info.controller, "EC 24 V, unit 0x0C")
        self.assertEqual(info.to_dict()["controller"], "EC 24 V, unit 0x0C")
        self.assertFalse(any("UNVERIFIED" in e or "family table" in e for e in info.errors), info.errors)
        # the family names of every EC unit code the driver has (display only)
        self.assertEqual(set(R.UNIT_CODES), {0x00, 0x01, 0x02, 0x03, 0x08, 0x09, 0x0A, 0x0B})
        self.assertEqual((R.UNIT_CODES[0x00], R.UNIT_CODES[0x02]), ("EC standard", "EC dust/drip-proof"))
        self.assertEqual(R.UNIT_CODES[0x0B], "EC(UC) ultra-ultra-compact")
        self.assertEqual(R.UNIT_CODES_CONFIRMED, frozenset())  # kept for compatibility; no longer used

    def test_cli_gripper_identify(self) -> None:
        out = io.StringIO()
        with contextlib.redirect_stderr(io.StringIO()):
            code = cli.main(["--sim", "--sim-profile", "gripper", "identify"], out=out)
        self.assertEqual(code, 0)
        text = out.getvalue()
        self.assertIn("model      EC-GRBP8M-20-1-1-PN-WA-WL2", text)
        self.assertIn('controller EC 24 V, unit 0x0A (EC(MS) "2-size EC")', text)
        self.assertIn("actuator   gripper (AKID 4)", text)
        self.assertIn("stroke     0.00..9.50 mm", text)
        self.assertIn("max speed  unit max 53.00 mm/s (MAXV 7.4); driver cap 53.00 mm/s", text)
        self.assertIn("max accel  unit max 0.30 / 0.30 G (MAXA / MAXD); driver cap 0.30 G", text)
        self.assertIn("push       50 % (recommended) .. 100 % (MXPP); driver cap 70 %", text)  # CLI default 70 %


# =========================================================================== 5. speed cap


class SpeedCapTest(unittest.TestCase):
    def test_gripper_refuses_more_than_53_mm_s(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=20.0)
        ec, rec = open_ec(sim)
        with ec:
            self.assertEqual(ec.speed_cap_mm_s, 100.0)  # not read yet
            with self.assertRaisesRegex(SoftLimitError, r"maximum speed 53\.00 mm/s \(MAXV item 7\.4"):
                ec.move_to(5.0, speed_mm_s=60.0)  # read lazily, then refused
            self.assertEqual(ec.speed_cap_mm_s, 53.0)
            self.assertEqual({f[1] for f in rec.tx}, {0x03}, "something other than a read was sent")
            for attempt in (lambda: ec.jog(-1.0, speed_mm_s=53.5),
                            lambda: ec.push(4.1, 5.0, 30, +1, speed_mm_s=54.0)):
                with self.assertRaises(SoftLimitError):
                    attempt()
            self.assertEqual(sim.alarm_code, 0)
            self.assertEqual(ec.move_to(5.0, speed_mm_s=53.0).position_mm, 5.0)

    def test_slider_cap_is_the_lower_of_both(self) -> None:
        ec, _ = open_ec(SimulatedEC(time_scale=50.0), max_speed_mm_s=1000.0)
        with ec:
            ec.identify()
            self.assertEqual(ec.speed_cap_mm_s, 700.0)
            with self.assertRaisesRegex(SoftLimitError, "700.00"):
                ec.move_to(100.0, speed_mm_s=750.0)
            self.assertEqual(ec.move_to(100.0, speed_mm_s=250.0, accel_g=0.3).position_mm, 100.0)
        ec, _ = open_ec(SimulatedEC(), max_speed_mm_s=100.0)
        with ec:
            ec.identify()
            self.assertEqual(ec.speed_cap_mm_s, 100.0)  # min(configured 100, 700)
        ec, _ = open_ec(SimulatedEC(), max_speed_mm_s=None)
        with ec:
            ec.identify()
            self.assertEqual(ec.speed_cap_mm_s, 700.0)  # nothing configured: the unit's own MAXV

    def test_maxv_unreadable_falls_back_to_0x0812(self) -> None:
        # MAXV (item 7.4) at 0x0706 not readable: the class-7 read fails
        ec, _ = open_ec(SimulatedEC(profile="gripper", time_scale=50.0, unreadable=[(0x0706, 2)]))
        with ec:
            info = ec.identify()
            self.assertEqual(info.max_speed_mm_s, 53.0)  # 8.10 VCMD, the default speed: equal to MAXV on the units tested
            self.assertIn("0x0812", ec.ec_max_speed_note)
            self.assertIsNone(info.max_accel_g)
            self.assertEqual(ec.accel_cap_g, 0.3)  # then only max_accel_g caps the acceleration

    def test_unreadable_speed_limit(self) -> None:
        # neither 0x0812 nor MAXV
        ec, _ = open_ec(SimulatedEC(time_scale=50.0, unreadable=[(R.SPEED_LIMIT, 2), (0x0706, 2)]))
        with ec:
            info = ec.identify()
            self.assertIsNone(info.max_speed_mm_s)
            self.assertIn("not readable", ec.ec_max_speed_note)
            self.assertTrue(any("speed limit" in e for e in info.errors))
            self.assertEqual(ec.speed_cap_mm_s, 100.0)
            ec.move_to(80.0, speed_mm_s=100.0)  # only the configured cap applies

    def test_position_table_speed_capped(self) -> None:
        sim = SimulatedEC(profile="gripper")
        ec, _ = open_ec(sim)
        with ec, self.assertRaisesRegex(SoftLimitError, "53.00"):
            ec.write_position_table(0, PositionEntry(position_mm=3.0, speed_mm_s=60.0), allow_nonvolatile=True)
        self.assertEqual(sim.nonvolatile_writes, 0)

    def test_cli_gripper_speed(self) -> None:
        for argv, code_expected in ((["move", "5", "--speed", "60"], 2), (["move", "5", "--speed", "50"], 0)):
            out, err = io.StringIO(), io.StringIO()
            with contextlib.redirect_stderr(err):
                code = cli.main(["--sim", "--sim-profile", "gripper", "--sim-time-scale", "20", *argv], out=out)
            self.assertEqual(code, code_expected, err.getvalue())
            if code:
                self.assertIn("53.00 mm/s", err.getvalue())


# =========================================================================== gripper profile end to end


class GripperProfileTest(unittest.TestCase):
    def test_profile_defaults(self) -> None:
        self.assertEqual(set(PROFILES), {"slider", "gripper", "rod", "pcon", "rotary", "unknown"})
        sim = SimulatedEC(profile="gripper")
        bus = ModbusRTU(sim, min_gap_s=0.0)
        self.assertEqual(bus.report_slave_id(1), bytes.fromhex("ecff0a0002"))
        self.assertEqual(bus.read_registers(1, R.SOFT_LIMITS, 4), [0, 980, 0xFFFF, 0xFFE2])  # +9.80 / -0.30
        self.assertEqual(bus.read_registers(1, R.SPEED_LIMIT, 2), [0, 5300])  # 53.00 mm/s
        self.assertEqual(bus.read_registers(1, 0x0000, 3), [0x0001, 0x0008, 0x0019])  # 25 records
        self.assertEqual(sim.dopm_layout, "2pos")
        with self.assertRaises(ValueError):
            SimulatedEC(profile="nope")
        with self.assertRaises(ValueError):
            SimulatedEC(dopm_layout="4pos")

    def test_check_passes_on_the_gripper(self) -> None:
        ec, _ = open_ec(SimulatedEC(profile="gripper", time_scale=20.0))
        with ec:
            report = run_check(ec, motion=True, port_label="sim")
        items = {item.name: item for item in report.items}
        self.assertTrue(report.passed, [(i.name, i.result, i.detail) for i in report.items])
        self.assertEqual(report.model, "EC-GRBP8M-20-1-1-PN-WA-WL2")
        self.assertEqual(report.controller, 'EC 24 V, unit 0x0A (EC(MS) "2-size EC")')
        self.assertIn("gripper (AKID 4)", items["slave_id"].detail)
        self.assertEqual(report.ec_max_speed_mm_s, 53.0)
        self.assertEqual(report.speed_cap_mm_s, 53.0)
        self.assertEqual(report.stroke, (0.0, 9.5))
        self.assertEqual(items["speed_limit"].result, PASS)
        self.assertIn("53.00 mm/s", items["speed_limit"].detail)
        self.assertEqual(items["index_table"].result, PASS)
        # the step adapts to the 9.50 mm stroke: a quarter of it (2.38 mm), toward the side with more room
        self.assertEqual(items["motion_out"].data["target_mm"], 7.12)
        self.assertEqual(items["motion_back"].data["reached_mm"], 9.5)

    def test_gripper_cycle(self) -> None:
        """The gripper sequence run on hardware, simulated: open, grip push (miss), STOP mid-move, home."""
        sim = SimulatedEC(profile="gripper", time_scale=20.0, pmsl=False)
        ec, _ = open_ec(sim)
        with ec:
            ec.connect()
            self.assertEqual(ec.effective_stroke, (0.0, 9.5))
            ec.move_to(3.0, speed_mm_s=45.0)
            result = ec.push(4.1, 5.4, 50, +1, speed_mm_s=20.0)  # row 1: 4.10 mm + 5.40 mm push
            self.assertTrue(result.missed)
            self.assertAlmostEqual(result.position_mm, 9.5, delta=0.01)
            st = ec.status()
            self.assertIs(st.at_forward_end, True)  # as at power-up after a missed grip
            with self.assertRaises(SoftLimitError):
                ec.move_to(9.6, speed_mm_s=10.0)  # inside the EC's +9.80 soft limit, outside its 9.50 stroke
            ec.move_to(6.0, speed_mm_s=10.0)
            ec.move_to(0.5, speed_mm_s=5.0, wait=False)
            time.sleep(0.03)  # ~0.6 s simulated: under way (the real gripper stopped 6.0 -> 0.5 at 3.90)
            st = ec.stop(wait=True)
            self.assertTrue(0.5 < st.position_mm < 5.95, st.position_mm)
            st = ec.home()
            self.assertEqual(st.position_mm, 0.0)


if __name__ == "__main__":
    unittest.main()
