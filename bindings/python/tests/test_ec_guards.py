"""Guards in ``iai_actuator_driver.ec``: the 0x45 home after a lost or refused reply, the tri-state safety speed of the
action mode, mm parameters on a rotary unit (in deg) and position-table writes on any unit code.

Everything runs against the simulator (no serial port is opened).
"""

from __future__ import annotations

import dataclasses
import struct
import unittest

from ec_testutil import FrameRecorder, hexbytes

from iai_actuator_driver import registers as R
from iai_actuator_driver import testing
from iai_actuator_driver.ec import Elecylinder, PositionEntry
from iai_actuator_driver.errors import IaiCommandRejected, ModbusTimeout, UnsafeCommandError, VerificationError
from iai_actuator_driver.modbus import ModbusRTU, build_frame
from iai_actuator_driver.sim import SimulatedEC

HOME_FRAME = hexbytes("01 45 00 0A 00 07 00 00 BC C3")  # 0x45 home, op1 = ORG 7 (VERIFIED on the EC-R6S rod)


def open_sim(sim: SimulatedEC, *, retries: int = 2) -> tuple[Elecylinder, FrameRecorder]:
    rec = FrameRecorder()
    ec = Elecylinder(ModbusRTU(sim, frame_logger=rec, min_gap_s=0.0, retries=retries), poll_interval_s=0.005,
                     owns_bus=True)
    return ec, rec


def sent(rec: FrameRecorder, function: int) -> list[bytes]:
    return [f for f in rec.tx if f[1] == function]


def home_frames(rec: FrameRecorder) -> list[bytes]:
    return [f for f in sent(rec, 0x45) if f[3] == R.IAI_CMD_HOME]


def stop_coil(rec: FrameRecorder) -> list[bool]:
    return [f[4] == 0xFF for f in sent(rec, 0x05) if struct.unpack(">H", f[2:4])[0] == R.COIL_STOP]


def is_home_frame(data: bytes) -> bool:
    return len(data) > 3 and data[1] == 0x45 and data[3] == R.IAI_CMD_HOME


# =========================================================================== 0x45 home: lost / refused replies


class HomeLostReplySim(SimulatedEC):
    """The EC runs the home frame, but its answer is lost on the line."""

    def on_request(self, frame: bytes) -> bytes | None:
        if is_home_frame(frame):
            self.handle(frame)
            return None
        return NotImplemented


class HomeFrameLostSim(SimulatedEC):
    """The home frame never reaches the EC (nothing runs, nothing is answered)."""

    def on_request(self, frame: bytes) -> bytes | None:
        return None if is_home_frame(frame) else NotImplemented


class HomeAlreadyRunningSim(SimulatedEC):
    """The home frame starts a home return, but the answer is rejection 0x24 (as a repeated frame would get)."""

    def on_request(self, frame: bytes) -> bytes | None:
        if is_home_frame(frame):
            self.handle(frame)
            return build_frame(self.slave, bytes([0x45, 0x24, 0]))
        return NotImplemented


class HomeRefusedSim(SimulatedEC):
    """The home frame is answered with rejection 0x11 and nothing runs."""

    def on_request(self, frame: bytes) -> bytes | None:
        if is_home_frame(frame):
            return build_frame(self.slave, bytes([0x45, 0x11, 0]))
        return NotImplemented


class IaiHomeRepliesTest(unittest.TestCase):
    def test_lost_reply_while_the_home_runs_is_waited_for(self) -> None:
        ec, rec = open_sim(HomeLostReplySim(profile="rod", time_scale=50.0))
        with ec:
            ec.move_to(30.0, speed_mm_s=50.0)
            rec.clear()
            with self.assertLogs("iai_actuator_driver.ec", "WARNING") as logs:
                st = ec.home(method="iai")
        self.assertTrue(any("seen running" in line for line in logs.output))
        self.assertTrue(st.homed and st.position_mm == 0.0)
        self.assertEqual(home_frames(rec), [HOME_FRAME])  # sent once: no retry that would be rejected 0x24
        self.assertEqual(stop_coil(rec), [])  # the home was seen running: it is this call's, nothing stopped

    def test_lost_reply_without_a_home_stops_and_raises(self) -> None:
        sim = HomeFrameLostSim(profile="rod", time_scale=50.0)
        ec, rec = open_sim(sim)
        with ec:
            ec.move_to(30.0, speed_mm_s=50.0)
            rec.clear()
            with (testing.timings(home_start_s=0.3), self.assertLogs("iai_actuator_driver.ec", "WARNING"),
                  self.assertRaises(ModbusTimeout) as caught):
                ec.home(method="iai")
            self.assertEqual(len(home_frames(rec)), 1)
            self.assertEqual(stop_coil(rec), [True, False])  # on doubt: decel stop (the frame may have run)
            notes = " ".join(caught.exception.__notes__)
            self.assertIn("no home return was seen", notes)
            self.assertIn("decel stop sent", notes)
            self.assertEqual(ec.status().position_mm, 30.0)

    def test_rejection_0x24_stops_the_running_home(self) -> None:
        sim = HomeAlreadyRunningSim(profile="rod", time_scale=2.0)
        ec, rec = open_sim(sim)
        with ec:
            ec.move_to(30.0, speed_mm_s=50.0)
            rec.clear()
            sim.events.clear()
            with self.assertRaises(IaiCommandRejected) as caught:
                ec.home(method="iai")
            self.assertEqual(caught.exception.rejection, 0x24)
            self.assertTrue(any("home return already in progress" in n for n in caught.exception.__notes__))
            self.assertEqual(stop_coil(rec), [True, False])  # never "nothing moves" while it homes
            events = list(sim.events)
            self.assertLess(events.index("home"), events.index("stop"))
            st = ec.status()
            self.assertFalse(st.homing or st.moving)
            self.assertFalse(st.homed)  # the home was cut short

    def test_answered_refusal_sends_no_stop(self) -> None:
        ec, rec = open_sim(HomeRefusedSim(profile="rod", time_scale=50.0))
        with ec:
            ec.move_to(30.0, speed_mm_s=50.0)
            rec.clear()
            with self.assertRaises(IaiCommandRejected) as caught:
                ec.home(method="iai")
            self.assertEqual(caught.exception.rejection, 0x11)
            self.assertEqual(len(home_frames(rec)), 1)
            self.assertEqual(stop_coil(rec), [])  # this frame did not run: nothing to stop
            self.assertFalse(ec.motion_pending)


# =========================================================================== action mode: tri-state safety speed


class ActionModeSafetyTest(unittest.TestCase):
    def test_pc_mode_keeps_the_present_safety_speed(self) -> None:
        sim = SimulatedEC(time_scale=50.0, pmsl=False, safety_speed=True)  # e.g. left on by IA-OS Teach mode
        ec, rec = open_sim(sim)
        with ec:
            # PC: the PMSL coil (PMSS 0 only) and the SFTY coil (only when it changes), never 0x45 op1 = 1 (on the
            # PCON-CB that left the servo refusing to come on; VERIFIED 2026-11-02); PIO stays 0x45 cmd 0x0C op1 = 0
            st = ec.set_action_mode(True)  # safety=None: keep SFTY as it is
            self.assertEqual((sent(rec, 0x45), sent(rec, 0x05)), ([], [hexbytes("01 05 04 27 FF 00 3D 01")]))
            self.assertTrue(st.modbus_enabled and st.safety_speed and sim.safety_speed)
            rec.clear()
            st = ec.set_action_mode(True, False)  # explicit off
            self.assertEqual((sent(rec, 0x45), sent(rec, 0x05)), ([], [hexbytes("01 05 04 01 00 00 9D 3A")]))
            self.assertFalse(st.safety_speed)
            rec.clear()
            st = ec.set_action_mode(True)  # off stays off: nothing written
            self.assertEqual((sent(rec, 0x45), sent(rec, 0x05)), ([], []))
            self.assertFalse(st.safety_speed)
            rec.clear()
            st = ec.set_action_mode(True, True)  # explicit on
            self.assertEqual((sent(rec, 0x45), sent(rec, 0x05)), ([], [hexbytes("01 05 04 01 FF 00 DC CA")]))
            self.assertTrue(st.safety_speed)
            rec.clear()
            st = ec.set_action_mode(False)  # PIO: the safety speed is kept too (op2 = 1)
            frame = sent(rec, 0x45)[0]
            self.assertEqual(frame[2:8], bytes([0x00, 0x0C, 0x00, 0x00, 0x00, 0x01]))
            self.assertFalse(st.modbus_enabled)
            self.assertTrue(sim.safety_speed)

    def test_pc_mode_without_coils_falls_back_to_the_command(self) -> None:
        """A unit that answers the PMSL coil with FC05 exception 01: PC by 0x45 cmd 0x0C op1 = 1, the one way left, and
        PMSS still read back. Once the bus knows FC05 is refused, no coil is tried at all."""
        sim = SimulatedEC(time_scale=50.0, pmsl=False, function_codes=(0x03, 0x06, 0x10, 0x11, 0x17, 0x45))
        ec, rec = open_sim(sim)

        def action_ops() -> list[bytes]:
            return [f[2:8] for f in sent(rec, 0x45) if f[3] == R.IAI_CMD_ACTION_MODE]

        with ec:
            self.assertFalse(ec.status().modbus_enabled)
            rec.clear()
            st = ec.set_action_mode(True, False)
            self.assertEqual(sent(rec, 0x05), [hexbytes("01 05 04 27 FF 00 3D 01")])  # tried once: exception 01
            self.assertEqual(action_ops(), [bytes([0x00, 0x0C, 0x00, 0x01, 0x00, 0x00])])
            self.assertTrue(st.modbus_enabled and sim.pmsl)
            ec.set_action_mode(False, False)
            rec.clear()
            st = ec.set_action_mode(True, True)  # FC05 known refused: straight to the command, op2 = the safety speed
            self.assertEqual(sent(rec, 0x05), [])
            self.assertEqual(action_ops(), [bytes([0x00, 0x0C, 0x00, 0x01, 0x00, 0x01])])
            self.assertTrue(st.modbus_enabled and st.safety_speed)
            ec.set_action_mode(False, False)
            sim.set_word_override(R.DSSE, ec.bus.read_registers(1, R.DSSE, 1)[0])  # op1 = 1 accepted, PMSS stays 0
            rec.clear()
            with self.assertRaisesRegex(VerificationError, "PMSS reads 0"):  # the fallback is verified too
                ec.set_action_mode(True, False)
            self.assertEqual(action_ops(), [bytes([0x00, 0x0C, 0x00, 0x01, 0x00, 0x00])])
            sim.set_word_override(R.DSSE, None)


# =========================================================================== rotary unit: mm parameters


class RotaryParameterTest(unittest.TestCase):
    def test_mm_parameters_are_written_in_degrees_on_a_rotary_unit(self) -> None:
        sim = SimulatedEC(time_scale=50.0, word_overrides={0x072F: 1})  # AKID (item 7.33, 0x072E-0x072F) = 1: rotary
        ec, rec = open_sim(sim)
        with ec:
            for no, value, kwargs in ((2, 0.2, {}), (4, 1.0, {"allow_home_params": True})):
                with self.subTest(no=no):
                    result = ec.write_user_parameter(no, value, commit=False, **kwargs)
                    self.assertEqual(result.unit, "deg")
                    self.assertTrue(any("deg on this rotary axis" in w for w in result.warnings))
            result = ec.write_user_parameter(5, 1, commit=False)  # a bit parameter: no unit at all
            self.assertEqual(result.after, "1:Enable")


# =========================================================================== position table: unknown unit code


def row_within_caps(ec: Elecylinder) -> PositionEntry:
    """Row 0 as the EC holds it, with speed / accel / decel inside the driver's default caps."""
    return dataclasses.replace(ec.read_position_table(0), speed_mm_s=50.0, accel_g=0.2, decel_g=0.2)


class PositionTableUnitCodeTest(unittest.TestCase):
    def test_unknown_unit_code_writes(self) -> None:
        ec, rec = open_sim(SimulatedEC(time_scale=50.0, unit_code=0x0C))
        with ec:
            entry = row_within_caps(ec)
            rec.clear()
            written = ec.write_position_table(0, entry, allow_nonvolatile=True)  # no per-model opt-in
            self.assertEqual(written.position_mm, entry.position_mm)
            self.assertEqual(len(sent(rec, 0x10)), 1)

    def test_unreadable_unit_code_writes(self) -> None:
        # FC11 too slow and CTCL (class 3 item 1, 0x0300) blank: the unit code cannot be read
        sim = SimulatedEC(time_scale=50.0, slave_id_delay_s=2.0, word_overrides={0x0300: 0, 0x0301: 0})
        ec, rec = open_sim(sim, retries=0)
        with ec:
            entry = row_within_caps(ec)
            rec.clear()
            ec.write_position_table(0, entry, allow_nonvolatile=True, allow_unverified_model=True)  # legacy flag: ignored
            self.assertEqual(len(sent(rec, 0x10)), 1)

    def test_known_unit_code_writes_without_the_opt_in(self) -> None:
        ec, rec = open_sim(SimulatedEC(profile="rod", time_scale=50.0))  # unit 0x00 (EC-R6S, also the EC-S6HH)
        with ec:
            entry = row_within_caps(ec)
            rec.clear()
            ec.write_position_table(0, entry, allow_nonvolatile=True)
            self.assertEqual(len(sent(rec, 0x10)), 1)


if __name__ == "__main__":
    unittest.main()
