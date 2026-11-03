"""Protocol facts the driver relies on: alarm levels, limits by item ID, IAI 0x45 command forms, identity.

Covered: alarm levels (message / operation cancelled / cold start) and the fault / warning rule, class-7 limits
by item ID (MAXV / MAXA / MAXD / MXPP / MNPP / TCPC / AKID), the 0x45 rejection table and never-send list, the
0x45 command forms (home 0x45 cmd 0x0A op1 = ORG, alarm reset 0x45 cmd 5 op1 0, action mode 0x45 cmd 0x0C, all
three VERIFIED on the EC-R6S rod), AXNO, CTCL, AKID, BRSL, MVPT CDAS. Where a test says UNVERIFIED, the behaviour is
expected but not yet confirmed on hardware. Everything runs against the simulator (no serial port is opened).
"""

from __future__ import annotations

import contextlib
import io
import unittest

from ec_testutil import FrameRecorder, hexbytes

from iai_actuator_driver import alarms as A
from iai_actuator_driver import cli
from iai_actuator_driver import registers as R
from iai_actuator_driver.ec import Elecylinder, Status
from iai_actuator_driver.errors import (
    EcAlarmError,
    EcNotReadyError,
    IaiCommandRejected,
    SoftLimitError,
    UnsafeCommandError,
)
from iai_actuator_driver.modbus import SLAVE_ID_TIMEOUT_S, ModbusRTU
from iai_actuator_driver.sim import SimulatedEC


def open_sim(profile: str = "slider", *, sim: SimulatedEC | None = None, max_accel: float = 0.3,
             **sim_kwargs: object) -> tuple[Elecylinder, SimulatedEC, FrameRecorder]:
    sim = sim or SimulatedEC(profile=profile, time_scale=50.0, **sim_kwargs)  # type: ignore[arg-type]
    rec = FrameRecorder()
    ec = Elecylinder(ModbusRTU(sim, frame_logger=rec, min_gap_s=0.0), max_accel_g=max_accel, poll_interval_s=0.005,
                     owns_bus=True)
    return ec, sim, rec


def sent(rec: FrameRecorder, function: int) -> list[bytes]:
    return [f for f in rec.tx if f[1] == function]


# =========================================================================== alarm levels


class AlarmLevelTest(unittest.TestCase):
    def test_levels_follow_the_alarm_code_ranges(self) -> None:
        for code in (0x047, 0x048, 0x049, 0x04A, 0x04E, 0x04F, 0x07F):
            self.assertEqual(A.alarm_level(code), A.LEVEL_MESSAGE, hex(code))
        self.assertEqual(A.alarm_level(0x011), A.LEVEL_UNKNOWN)  # 0x011-0x07F but not in MESSAGE_CODES
        for code in (0x080, 0x082, 0x08A, 0x090, 0x096, 0x0A2, 0x0A5, 0x0A7, 0x0AB, 0x0AE, 0x0B5, 0x0B6, 0x0BA,
                     0x0BE, 0x0C2, 0x0C5, 0x0CF, 0x0D6, 0x0D8, 0x0DC, 0x0DE, 0x0DF, 0x0E1, 0x0ED, 0x0EF):
            self.assertEqual(A.alarm_level(code), A.LEVEL_OPERATION_CANCELLED, hex(code))
        for code in (0x081, 0x08B, 0x097, 0x0A0, 0x0A1, 0x0A6, 0x0B8, 0x0C8, 0x0D5, 0x0E0, 0x0E5, 0x0F8, 0x0FD):
            self.assertEqual(A.alarm_level(code), A.LEVEL_COLD_START, hex(code))
        # device-dependent: 0x0EE operation cancelled on absolute (WA) ECs, cold start without; 0x0CE / 0x0D2
        # (supply voltage) are cold start on every unit (conservative)
        self.assertEqual(A.alarm_level(0x0EE), A.LEVEL_OPERATION_CANCELLED)
        self.assertEqual(A.alarm_level(0x0EE, absolute_encoder=False), A.LEVEL_COLD_START)
        for system in (0xEC, 0xED):
            self.assertEqual(A.alarm_level(0x0D2, system_code=system), A.LEVEL_COLD_START)
            self.assertEqual(A.alarm_level(0x0CE, system_code=system), A.LEVEL_COLD_START)
        self.assertEqual((A.alarm_level(0), A.alarm_level(R.ALARM_POWER_UP)), (A.LEVEL_NONE, A.LEVEL_NONE))
        self.assertEqual(A.alarm_level(0xE30), A.LEVEL_UNKNOWN)

    def test_groups_and_info(self) -> None:
        self.assertEqual([A.alarm_group(c) for c in (0x04E, 0x0C1, 0x0EE, 0x0A2, 0x090)],
                         ["Warning", "A", "D", "F", "Other"])
        info = A.alarm_info(0x04E)
        self.assertEqual((info["level"], info["fault"], info["warning"], info["group_name"]),
                         ("message", False, True, "maintenance warning"))
        self.assertIn("keeps running", info["remedy"])
        self.assertIn("home return", A.alarm_info(0x0EE)["home_needed"])
        self.assertIn("2 s", A.alarm_info(0x0C1)["remedy"])  # axis blocked after a move command
        self.assertIn("software reset", A.alarm_info(0x0E0)["reset"])  # cold start
        self.assertEqual(A.describe_alarm_detail(0x0A2, 2, 0x1024), "row 2, field speed")

    def test_status_fault_and_warning_rule(self) -> None:
        def st(code: int, dss1: int = 0x3098) -> Status:
            return Status.from_registers([0, 7395, code, 0, 0, dss1, 0x8000, 0x33C2])

        warn = st(0x04E)  # a message code WITHOUT ALML is still a warning (UNVERIFIED: the axis keeps running)
        self.assertEqual((warn.alarm, warn.warning, warn.fault, warn.alarm_level), (False, True, False, "message"))
        self.assertTrue(warn.ready)  # never blocks motion
        self.assertFalse(st(0x04E, 0x3098 | R.DSS1_ALMH).warning)  # ALMH always makes it a fault
        self.assertTrue(st(0x04E, 0x3098 | R.DSS1_ALMH).alarm)
        oc = st(0x083)  # operation cancelled, even without ALMH
        self.assertEqual((oc.alarm, oc.warning, oc.ready), (True, False, False))
        self.assertTrue(st(0xE30).alarm)  # unknown level: fail safe ...
        self.assertFalse(st(0xE30, 0x3098 | R.DSS1_ALML).alarm)  # ... unless only the warning bit is set
        moving = Status.from_registers([0, 7395, 0x04E, 0, 0, 0x3090 | R.DSS1_ALML, 0x8000, 0x33C2])
        self.assertTrue(moving.moving)  # a warning does not make a running axis look settled


class WarningKeepsRunningTest(unittest.TestCase):
    def test_move_completes_with_a_maintenance_warning(self) -> None:
        ec, sim, _ = open_sim()
        with ec:
            sim.trigger_alarm(0x04E)  # ALML, servo stays on
            st = ec.move_to(90.0, speed_mm_s=50.0)  # neither refused nor stopped by the warning
            self.assertEqual(st.position_mm, 90.0)
            self.assertTrue(st.warning and not st.alarm)
            ec.move_to(80.0, speed_mm_s=50.0, wait=False)
            sim.trigger_alarm(0x0C1)  # operation cancelled mid-move: a fault
            with self.assertRaises(EcAlarmError):
                ec.wait_idle(target_mm=80.0, timeout=5.0)


# =========================================================================== limits from class 7


class AxisLimitsTest(unittest.TestCase):
    def test_limits_by_item_id_on_each_profile(self) -> None:
        expected = {  # (MAXV, MAXA, MAXD, MXPP, MNPP or 20, AKID): the simulator's profiles of the units tested
            "slider": (700.0, 1.0, 1.0, 60.0, 20.0, 9),
            "gripper": (53.0, 0.3, 0.3, 100.0, 50.0, 4),
            "rod": (800.0, 1.0, 1.0, 70.0, 20.0, 3),
        }
        for profile, values in expected.items():
            with self.subTest(profile=profile):
                ec, _, _ = open_sim(profile)
                with ec:
                    lim = ec.read_axis_limits()
                self.assertEqual((lim.max_speed_mm_s, lim.max_accel_g, lim.max_decel_g, lim.max_push_pct,
                                  lim.min_push_pct, lim.actuator_kind), values)
                self.assertTrue(lim.push_available)
                self.assertEqual(lim.min_push_from_ec, profile != "rod")  # the rod has no MNPP: default 20 %
                self.assertIn("MAXV item 7.4", ec.ec_max_speed_note)

    def test_accel_capped_by_maxa_maxd(self) -> None:
        ec, sim, rec = open_sim("gripper", max_accel=1.0)  # the configured cap is above the gripper's 0.30 G
        with ec:
            ec.identify()
            self.assertEqual(ec.accel_cap_g, 0.3)
            rec.clear()
            with self.assertRaisesRegex(SoftLimitError, r"exceeds this EC's maximum 0\.30 G \(MAXA 7\.5\)"):
                ec.move_to(5.0, speed_mm_s=20.0, accel_g=0.31)
            self.assertEqual(sent(rec, 0x10), [])
            self.assertEqual(sim.alarm_code, 0)
            self.assertEqual(ec.move_to(5.0, speed_mm_s=20.0, accel_g=0.3).position_mm, 5.0)

    def test_rod_row_decel_within_maxd_runs_with_allow_above_caps(self) -> None:
        ec, _, _ = open_sim("rod")  # row 0: 240 mm/s, accel 0.10 / decel 0.47 G (the EC-R6S rod)
        ec.max_speed_mm_s = 100.0  # an integrator's cap (the default is the unit's own MAXV)
        with ec:
            ec.identify()
            ec.move_to(50.0, speed_mm_s=50.0)
            with self.assertRaisesRegex(SoftLimitError, "speed cap"):
                ec.move_to_position(0)  # above the configured caps (100 mm/s, 0.3 G)
            st = ec.move_to_position(0, allow_above_caps=True)  # within MAXV 800 / MAXD 1.00 G: the EC's own values
            self.assertEqual(st.position_mm, 0.0)

    def test_rotary_unit_works_in_degrees(self) -> None:
        # AKID (item 7.33, 0x072E on the slider) = 1: a rotary actuator
        ec, _, rec = open_sim(sim=SimulatedEC(time_scale=50.0, word_overrides={0x072F: 1}))
        with ec:
            info = ec.identify()
            self.assertTrue(info.rotary)
            self.assertEqual((info.units, info.kind, ec.units, ec.kind), ("deg", "rotary", "deg", "rotary"))
            self.assertTrue(any("rotary" in e and "deg" in e for e in info.errors))
            self.assertEqual(ec.move_to(80.0, speed_mm_s=20.0).position_mm, 80.0)  # the same commands, in degrees
            self.assertEqual(ec.move_to_position(2, allow_above_caps=True).position_mm, 75.0)


# =========================================================================== IAI 0x45 command forms


class IaiCommandFormsTest(unittest.TestCase):
    def test_home_with_op1_org(self) -> None:
        ec, sim, rec = open_sim("rod")
        with ec:
            ec.move_to(30.0, speed_mm_s=50.0)
            self.assertEqual(ec.read_home_pattern(), 7)  # ORG item 9.16 (the EC-R6S rod: 0x0007 at 0x091E)
            rec.clear()
            st = ec.home(method="iai")
            self.assertTrue(st.homed and st.position_mm == 0.0)
            self.assertIn(hexbytes("01 45 00 0A 00 07 00 00 BC C3"), rec.tx)  # the frame VERIFIED on the EC-R6S rod
            self.assertEqual([f for f in sent(rec, 0x05) if f[2:4] == bytes([0x04, 0x0B])], [])  # no HOME coil

    def test_home_without_a_valid_org_is_refused(self) -> None:
        ec, _, rec = open_sim(sim=SimulatedEC(time_scale=50.0, word_overrides={0x091E: 0, 0x091F: 0}))  # slider ORG = 0
        with ec:
            rec.clear()
            with self.assertRaisesRegex(EcNotReadyError, "home pattern ORG"):
                ec.home(method="iai")
            self.assertEqual(sent(rec, 0x45), [])

    def test_home_op1_zero_is_rejected_with_its_reason(self) -> None:
        bus = ModbusRTU(SimulatedEC(), min_gap_s=0.0)
        with self.assertRaises(IaiCommandRejected) as caught:
            bus.iai_command(1, 0, R.IAI_CMD_HOME, 0)  # rejected 0x11 (VERIFIED on the gripper)
        self.assertEqual((caught.exception.rejection, caught.exception.reason), (0x11, "1st operand error"))
        self.assertIn("ORG", caught.exception.hint)

    def test_alarm_reset_with_0x45(self) -> None:
        ec, sim, rec = open_sim()
        with ec:
            sim.trigger_alarm(0x0C1)
            rec.clear()
            st = ec.reset_alarm(method="iai")
            self.assertEqual(st.alarm_code, 0)
            self.assertIn(hexbytes("01 45 00 05 00 00 00 00 59 03"), rec.tx)
            self.assertEqual([f for f in sent(rec, 0x05) if f[2:4] == bytes([0x04, 0x07])], [])  # no ALRS coil
        with self.assertRaises(ValueError):
            ec.reset_alarm(method="x")

    def test_action_mode(self) -> None:
        ec, sim, rec = open_sim(pmsl=False)
        with ec:
            # PIO: the 0x45 cmd 0x0C frame VERIFIED on the rod; PC: the PMSL / SFTY coils, not op1 = 1 (on the PCON-CB
            # op1 = 1 left the servo refusing to come on, VERIFIED 2026-11-02)
            for (pc, safety), frames in (((True, True), ([], ["01 05 04 27 FF 00 3D 01", "01 05 04 01 FF 00 DC CA"])),
                                         ((True, False), ([], ["01 05 04 01 00 00 9D 3A"])),
                                         ((False, False), (["01 45 00 0C 00 00 00 00 85 02"], []))):
                with self.subTest(pc=pc, safety=safety):
                    rec.clear()
                    st = ec.set_action_mode(pc, safety)
                    self.assertEqual((sent(rec, 0x45), sent(rec, 0x05)),
                                     ([hexbytes(f) for f in frames[0]], [hexbytes(f) for f in frames[1]]))
                    self.assertEqual((st.modbus_enabled, sim.pmsl), (pc, pc))
                    if pc:
                        self.assertEqual(st.safety_speed, safety)
            ec.set_action_mode(True)
            ec.move_to(100.0, speed_mm_s=20.0, wait=False)
            rec.clear()
            with self.assertRaisesRegex(EcNotReadyError, "at rest"):
                ec.set_action_mode(False)
            self.assertEqual(sent(rec, 0x45), [])
            ec.stop(wait=True)

    def test_cli_mode_and_methods(self) -> None:
        def run(*argv: str) -> tuple[int, str]:
            out = io.StringIO()
            with contextlib.redirect_stderr(io.StringIO()):
                code = cli.main(list(argv), out=out)
            return code, out.getvalue()

        code, out = run("--sim", "mode", "pc", "--safety")
        self.assertEqual(code, 0, out)
        self.assertIn("SAFETY-SPEED", out)
        code, out = run("--sim", "--sim-time-scale", "50", "home", "--method", "iai")
        self.assertEqual(code, 0, out)
        self.assertIn("HOMED", out)
        code, out = run("--sim", "alarm-reset", "--method", "iai")
        self.assertEqual(code, 0, out)


# =========================================================================== identity, link, unit info


class IdentityAndLinkTest(unittest.TestCase):
    def test_fc11_timeout_covers_a_slow_reply(self) -> None:
        self.assertGreaterEqual(SLAVE_ID_TIMEOUT_S, 0.65)  # the EC-R6S rod answered FC11 after 0.2-0.4 s (VERIFIED)

    def test_software_reset_probes_brsl_first(self) -> None:
        # after a software reset the EC answers at its power-up rate (38400, VERIFIED on the EC-R6S rod), which
        # the driver reads from BRSL (item 8.3 = 6 = 38400) and tries first
        ec, sim, _ = open_sim("gripper", baudrate=115200, reset_baudrate=38400)
        with ec:
            self.assertEqual(ec.read_boot_baudrate(), 38400)
            sim.events.clear()
            result = ec.software_reset(reset_wait_s=0.1)
            self.assertEqual((result.baudrate, ec.baudrate), (38400, 38400))
            after = list(sim.events)[list(sim.events).index("software reset"):]
            self.assertFalse(any("not understood" in e for e in after), after)  # 38400 tried first: no miss

    def test_unit_info_capabilities(self) -> None:
        ec, _, _ = open_sim("slider", collision_detection=True)
        with ec:
            data = ec.unit_info(sections=["identity", "monitor"])
        ident = data["identity"]
        self.assertEqual((ident["actuator"], ident["axis_number"], ident["identity_source"]),
                         ("slider (small) (AKID 9)", 0, "FC11"))
        caps = ident["capabilities"]
        flags = caps["support"]["flags"]
        self.assertTrue(flags["push"] and flags["relative_moves"] and flags["software_reset"])
        self.assertFalse(flags["uses_function_0x48"])
        self.assertTrue(caps["collision"]["supported"])  # MVPT b13 CDAS
        self.assertEqual(caps["collision"]["parameter_items"], [])  # no class-8 items 119-127 on this directory
        self.assertEqual((caps["link"]["boot_baudrate"], caps["link"]["transmit_delay_ms"]), (38400, 5))
        self.assertEqual((caps["servo_monitor"]["mode"], caps["servo_monitor"]["mode_name"]), (0, "off"))
        self.assertEqual(caps["axis_limits"]["push_force_at_max_estimate"], 66.0)  # TCPC 66 at MXPP 60 %
        self.assertEqual(caps["push_speed_mm_s"], 30.0)  # PSHV 8.15
        self.assertEqual(data["monitor"]["rated_current"], 1200)  # MCUR item 4.5
        self.assertIn("current_pct", data["monitor"])
        ec, _, _ = open_sim("rod")
        with ec:
            caps = ec.unit_info(sections=["identity"])["identity"]["capabilities"]
        self.assertFalse(caps["collision"]["supported"])  # the EC-R6S rod: MVPT 0x0317, CDAS = 0 (VERIFIED)
        self.assertFalse(caps["support"]["flags"]["has_sup3"])  # 6 class-1 members

    def test_p3_write_flips_the_pattern_parity(self) -> None:
        ec, sim, _ = open_sim("gripper")
        with ec:
            result = ec.write_user_parameter(3, "0:Reverse", allow_home_params=True, commit=False)
            self.assertEqual((result.raw_before, result.raw_after, result.after), ("00000007", "00000008", "0:Reverse"))
            self.assertTrue(any("home pattern ORG 7 -> 8" in w for w in result.warnings))
            result = ec.write_user_parameter(3, "1:Forward", allow_home_params=True, commit=False)
            self.assertEqual((result.raw_after, result.after), ("00000007", "1:Forward"))
        self.assertEqual(sim.parameter_commits, 0)


if __name__ == "__main__":
    unittest.main()
