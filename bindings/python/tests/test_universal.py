"""Universal units, ranges and features: one generic axis model for every IAI unit (simulator only).

The PCON-CB + RCP6 profile (a CON-family controller, VERIFIED to speak the same protocol), a rotary EC (AKID 1,
degrees) and an unknown unit must all work with the same commands; ranges come from the unit (limits()); the
command-profile move (FC17 0x0F00 + PCRF), the 0x45 cmd 6 speed override, the CON software reset, generic and
named parameters and read_monitor().
"""

from __future__ import annotations

import json
import struct
import unittest

from ec_testutil import FrameRecorder

from iai_actuator_driver import con_params
from iai_actuator_driver import registers as R
from iai_actuator_driver.ec import Axis, Elecylinder, PositionEntry
from iai_actuator_driver.errors import SoftLimitError, UnsafeCommandError
from iai_actuator_driver.modbus import ModbusRTU
from iai_actuator_driver.sim import SimulatedEC


def open_axis(profile: str = "pcon", **kwargs: object) -> tuple[Elecylinder, SimulatedEC, FrameRecorder]:
    sim = SimulatedEC(profile=profile, time_scale=50.0, **kwargs)  # type: ignore[arg-type]
    rec = FrameRecorder()
    ec = Elecylinder(ModbusRTU(sim, frame_logger=rec, min_gap_s=0.0), poll_interval_s=0.005, owns_bus=True)
    return ec, sim, rec


def sent(rec: FrameRecorder, function: int) -> list[bytes]:
    return [f for f in rec.tx if f[1] == function]


def coil_writes(rec: FrameRecorder, coil: int) -> list[bool]:
    return [f[4:6] == b"\xff\x00" for f in sent(rec, 0x05) if struct.unpack(">H", f[2:4])[0] == coil]


class TextHelpersTest(unittest.TestCase):
    def test_strings_are_printable_ascii(self) -> None:
        words = [0x5043, 0x4F4E, 0x00B8, 0xDC0A, 0x2020]
        self.assertEqual(R.words_to_ascii(words), "PCON")
        self.assertIsNone(R.words_to_text(words))  # binary data is not a model string
        self.assertEqual(R.words_to_text([0x4543, 0x2D52, 0x3600, 0x0000]), "EC-R6")
        self.assertEqual(R.printable("café\n"), "caf")

    def test_family_and_units(self) -> None:
        self.assertEqual([R.product_family(c) for c in (0xEC, 0xED, 0xAE, 0xAC, 0xAA, 0x99, None)],
                         ["EC", "EC", "PCON", "ACON/DCON", "SCON", "IAI controller", "IAI controller"])
        self.assertEqual([R.axis_units(k) for k in (1, 0, 3, 42, None)], ["deg", "mm", "mm", "mm", "mm"])
        self.assertEqual(R.axis_kind_name(42), "kind 42")
        self.assertIs(Axis, Elecylinder)


class PconTest(unittest.TestCase):
    """PCON-CB + RCP6-WSA10R: identity, ranges, motion, features (all VERIFIED on the PCON-CB with an RCP6)."""

    def test_identify(self) -> None:
        ec, _, _ = open_axis()
        with ec:
            info = ec.identify()
        self.assertEqual((info.system_code, info.unit_code, info.product_family), (0xAE, 0x44, "PCON"))
        self.assertEqual((info.model, info.controller_name, info.model_name), ("PCON-CB (FB)", "PCON-CB (FB)", None))
        self.assertEqual(info.serial_number, "SIM-0000005")
        self.assertEqual((info.units, info.kind, info.max_positions), ("mm", "unknown", 64))
        self.assertEqual(info.max_speed_mm_s, 840.0)
        text = json.dumps(info.to_dict())
        text.encode("cp1252")  # never a crash on a cp1252 console
        self.assertTrue(all(0x20 <= ord(c) <= 0x7E for e in info.errors for c in e))

    def test_limits_come_from_the_unit(self) -> None:
        ec, _, _ = open_axis()
        with ec:
            limits = ec.limits()
        json.dumps(limits)
        self.assertEqual((limits["family"], limits["units"]), ("PCON", "mm"))
        self.assertEqual((limits["position"]["min"], limits["position"]["max"]), (0.0, 150.0))
        self.assertEqual((limits["speed"]["max"], limits["speed"]["unit_max"], limits["speed"]["configured"]),
                         (840.0, 840.0, None))
        self.assertEqual((limits["accel"]["max"], limits["decel"]["max"]), (1.0, 1.0))
        self.assertEqual((limits["stored_rows"]["min"], limits["stored_rows"]["max"]), (0, 63))
        source = "0x45 cmd 6 op2 = round(256 x % / 100) - 1 (the unit runs (op2 + 1) / 256)"
        self.assertEqual(limits["speed_override"], {"min": 1, "max": 100, "unit": "%", "source": source})

    def test_configured_cap_is_the_integrators(self) -> None:
        sim = SimulatedEC(profile="pcon", time_scale=50.0)
        ec = Elecylinder(ModbusRTU(sim, min_gap_s=0.0), max_speed_mm_s=200.0, poll_interval_s=0.005, owns_bus=True)
        with ec:
            self.assertEqual(ec.limits()["speed"]["max"], 200.0)
            with self.assertRaises(SoftLimitError):
                ec.move_to(100.0, speed_mm_s=250.0)

    def test_moves_with_the_rtc_warning_active(self) -> None:
        ec, sim, _ = open_axis()
        with ec:
            st = ec.status()
            self.assertEqual(st.alarm_code, R.ALARM_RTC_STOP)
            self.assertTrue(st.warning and not st.alarm)  # message level: the axis still runs
            self.assertEqual(ec.move_to(100.0, speed_mm_s=200.0, accel_g=0.3).position_mm, 100.0)
            st = ec.reset_alarm()
            self.assertEqual(st.alarm_code, R.ALARM_RTC_STOP)  # comes back while the clock is unset
            self.assertEqual(ec.move_to(20.0, speed_mm_s=200.0, accel_g=0.3).position_mm, 20.0)

    def test_profile_move_with_its_own_deceleration(self) -> None:
        ec, sim, rec = open_axis()
        with ec:
            ec.identify()
            rec.clear()
            st = ec.move_to(120.0, speed_mm_s=100.0, accel_g=0.3, decel_g=0.1)
            self.assertEqual(st.position_mm, 120.0)
        self.assertEqual(sim.profile_moves, 1)
        fc17 = sent(rec, 0x17)
        self.assertEqual(len(fc17), 1)
        r_addr, r_count, w_addr, w_count = struct.unpack(">HHHH", fc17[0][2:10])
        self.assertEqual((r_addr, r_count, w_addr, w_count), (0x9000, 34, 0x0F00, 20))
        words = struct.unpack(">20H", fc17[0][11:51])
        self.assertEqual(R.from_i32_words(*words[0:2]), 12000)  # PCMD 120.00
        self.assertEqual((R.from_u32_words(*words[4:6]), R.from_u32_words(*words[6:8])), (30, 10))  # ACMD / DCMD
        self.assertEqual((words[10], words[11]), (0x59, 0xFF))  # SPOW / DPOW kept from the unit's block
        self.assertEqual(words[12:], (0,) * 8)  # PPOW, CTLF, LPOW, CTF2, zones 0
        self.assertEqual(coil_writes(rec, R.COIL_PCRF), [True])  # ON only: the coil clears itself
        self.assertEqual(sent(rec, 0x10), [])  # not the 0x9900 path

    def test_profile_move_retarget_and_stop(self) -> None:
        ec, sim, _ = open_axis()
        with ec:
            ec.move_to(140.0, speed_mm_s=50.0, accel_g=0.3, decel_g=0.2, wait=False)
            self.assertTrue(sim.moving)
            st = ec.move_to(70.0, speed_mm_s=50.0, accel_g=0.3, decel_g=0.2)  # a new profile + PCRF retargets
            self.assertEqual(st.position_mm, 70.0)
            ec.move_to(140.0, speed_mm_s=50.0, accel_g=0.3, decel_g=0.2, wait=False)
            ec.stop()
            self.assertFalse(sim.moving)
            self.assertLess(ec.status().position_mm, 140.0)

    def test_profile_move_checks_maxd_first(self) -> None:
        ec, sim, rec = open_axis()
        with ec:
            ec.identify()
            rec.clear()
            with self.assertRaisesRegex(SoftLimitError, "MAXD"):
                ec.move_to(100.0, speed_mm_s=50.0, accel_g=0.3, decel_g=1.5)
            self.assertEqual(sent(rec, 0x17) + sent(rec, 0x05), [])

    def test_stored_rows_beyond_two_and_speed_override(self) -> None:
        ec, sim, rec = open_axis()
        with ec:
            self.assertEqual(ec.max_positions(), 64)
            self.assertEqual(ec.move_to_position(6).position_mm, 48.21)  # row 6: 48.21 mm
            rec.clear()
            st = ec.move_to_position(1, speed_pct=25)
            self.assertEqual(st.position_mm, 87.90)
            frames = sent(rec, 0x45)
            self.assertEqual(len(frames), 1)
            self.assertEqual(frames[0][2:8], bytes([0, R.IAI_CMD_MOVE_POSITION, 0, 1, 0, 63]))  # 256 x 25 % - 1
            self.assertEqual(sim.override_moves, 1)
            with self.assertRaises(SoftLimitError):
                ec.move_to_position(64)  # beyond MAXP
            with self.assertRaises(SoftLimitError):
                ec.move_to_position(1, speed_pct=0)

    def test_position_table_row_beyond_two(self) -> None:
        ec, sim, _ = open_axis()
        with ec:
            entry = PositionEntry(position_mm=33.32, speed_mm_s=100.0, accel_g=0.3, decel_g=0.3)
            written = ec.write_position_table(10, entry, allow_nonvolatile=True)
            self.assertEqual(written.position_mm, 33.32)
            self.assertTrue(written.checksum_ok)
            self.assertEqual(ec.move_to_position(10).position_mm, 33.32)

    def test_software_reset_is_confirmed_by_the_reply_gap(self) -> None:
        ec, sim, _ = open_axis(reset_s=10.0)  # 0.2 s of silence at time_scale 50
        with ec:
            ec.write_parameter(3, 140.0, commit=False)  # RAM only: the reset reloads 150.30 from non-volatile memory
            self.assertEqual(ec.read_parameter(3)["value"], 140.0)
            result = ec.software_reset(reset_wait_s=0.5)
            self.assertTrue(result.confirmed, result.notes)
            self.assertIn("silent", result.confirmed_by)
            self.assertTrue(result.modbus_enabled)  # the PCON keeps PMSS 1
            self.assertEqual(sim.software_resets, 1)
            self.assertEqual(ec.read_parameter(3)["value"], 150.3)

    def test_named_con_parameters(self) -> None:
        ec, sim, _ = open_axis()
        with ec:
            params = {p["number"]: p for p in ec.parameters()}
            self.assertNotIn(188, params)  # its item is not in this unit's ID map
            self.assertEqual(len(params), len(con_params.CON_PARAMETERS) - 1)
            self.assertEqual((params[3]["text"], params[3]["item"], params[3]["dangerous"]), ("150.30 mm", "7.1", False))
            self.assertEqual((params[16]["value"], params[16]["dangerous"]), (38400, True))
            self.assertEqual(params[5]["text"], "0:Reverse")  # home pattern 8: even
            self.assertTrue(all(set(p) >= {"number", "name", "item", "value", "unit", "min", "max", "writable",
                                           "dangerous"} for p in params.values()))
            result = ec.write_parameter(35, 200)  # safety velocity: SAFE, committed
            self.assertEqual((result.after, result.committed, result.commit_confirmed), ("200.00 mm/s", True, True))
            self.assertEqual(sim.parameter_commits, 1)
            baud = ec.write_parameter(16, 115200, commit=False)  # baud: dangerous, written with a warning
            self.assertEqual(baud.after, "115200 bps")
            self.assertTrue(any("is a dangerous parameter" in w for w in baud.warnings), baud.warnings)
            with self.assertRaises(SoftLimitError):
                ec.write_parameter(35, 251)  # outside 1..250
            wider = ec.write_parameter(3, 151.0, commit=False)  # widening a soft limit is a stroke change: a warning
            self.assertTrue(any("widens" in w for w in wider.warnings), wider.warnings)
            self.assertEqual(ec.write_parameter("No.3", 140.0, commit=False).after, "140.00 mm")
            self.assertEqual(ec.effective_stroke, (0.0, 139.7))  # enforced at once

    def test_raw_items(self) -> None:
        ec, sim, _ = open_axis()
        with ec:
            raw = ec.read_parameter("8.15")
            self.assertEqual((raw["named"], raw["dangerous"]), (34, False))
            self.assertEqual(ec.write_parameter("8.15", 3000, commit=False).after, "00000BB8")
            gain = ec.write_parameter("9.7", 5, commit=False)  # a servo gain: written with a warning
            self.assertTrue(any("written anyway" in w for w in gain.warnings), gain.warnings)
            with self.assertRaisesRegex(UnsafeCommandError, "parameter class"):
                ec.write_parameter("3.1", 0, allow_dangerous=True)
            with self.assertRaises(UnsafeCommandError):
                ec.read_parameter("8.999")

    def test_read_monitor(self) -> None:
        ec, _, _ = open_axis()
        with ec:
            m = ec.read_monitor()
        json.dumps(m)
        self.assertEqual((m["position"], m["alarm"], m["alarm_level"], m["units"]), (60.0, 0x069, "message", "mm"))
        self.assertEqual(len(m["words"]), 34)
        self.assertIsNotNone(m["current_pct"])
        self.assertIn("DEVI", m["raw"])

    def test_unit_info(self) -> None:
        ec, _, _ = open_axis()
        with ec:
            data = ec.unit_info(sections=["identity", "parameters"])
        json.dumps(data)
        self.assertEqual((data["identity"]["product_family"], data["identity"]["model"]), ("PCON", "PCON-CB (FB)"))
        self.assertEqual(data["parameters"]["named"], [])  # the EC P-numbers do not apply
        self.assertTrue(any(p["number"] == 3 for p in data["parameters"]["numbered"]))


class RotaryTest(unittest.TestCase):
    def test_a_rotary_axis_works_in_degrees(self) -> None:
        ec, sim, _ = open_axis("rotary")
        with ec:
            info = ec.identify()
            self.assertEqual((info.units, info.kind, info.rotary, info.model), ("deg", "rotary", True,
                                                                                "SIM-EC-ROTARY-360"))
            limits = ec.limits()
            self.assertEqual((limits["units"], limits["position"]["max"], limits["speed"]["unit"]),
                             ("deg", 360.0, "deg/s"))
            self.assertEqual(ec.move_to(270.0, speed_mm_s=300.0, accel_g=0.3).position_mm, 270.0)
            self.assertEqual(ec.move_to_position(1).position_mm, 90.0)
            self.assertEqual(ec.move_to(45.0, speed_mm_s=100.0, accel_g=0.3, decel_g=0.1).position_mm, 45.0)
            with self.assertRaises(SoftLimitError):
                ec.move_to(361.0)
            row = ec.write_position_table(1, PositionEntry(position_mm=180.0, speed_mm_s=100.0, accel_g=0.3,
                                                           decel_g=0.3), allow_nonvolatile=True)  # fmt: skip
            self.assertEqual(row.position_mm, 180.0)
            params = {p["number"]: p for p in ec.parameters()}
            self.assertEqual(params[1]["unit"], "deg")
            result = ec.write_user_parameter(2, 0.2, commit=False)
            self.assertEqual(result.unit, "deg")


class UnknownUnitTest(unittest.TestCase):
    def test_an_unknown_unit_works(self) -> None:
        ec, sim, _ = open_axis("unknown")
        with ec:
            info = ec.identify()
            self.assertEqual((info.product_family, info.unit_known, info.model_name), ("IAI controller", False, None))
            self.assertEqual(info.model, "system code 0x99, unit 0x7E")
            self.assertEqual((info.units, info.kind), ("mm", "kind 42"))
            limits = ec.limits()
            self.assertEqual((limits["position"]["max"], limits["speed"]["max"]), (100.0, 400.0))
            ec.home()
            self.assertEqual(ec.move_to(80.0, speed_mm_s=200.0, accel_g=0.3).position_mm, 80.0)
            self.assertEqual(ec.move_to(10.0, speed_mm_s=200.0, accel_g=0.3, decel_g=0.2).position_mm, 10.0)
            self.assertEqual(ec.move_to_position(1).position_mm, 90.0)
            self.assertTrue(ec.parameters())  # offered by item ID
            m = ec.read_monitor()
            self.assertEqual(m["position"], 90.0)


class EcFeaturesTest(unittest.TestCase):
    def test_profile_move_on_an_ec(self) -> None:  # UNVERIFIED on an EC; modelled like the PCON
        ec, sim, rec = open_axis("rod")
        with ec:
            st = ec.move_to(100.0, speed_mm_s=200.0, accel_g=0.5, decel_g=0.2)
        self.assertEqual(st.position_mm, 100.0)
        self.assertEqual((sim.profile_moves, len(sent(rec, 0x17))), (1, 1))

    def test_default_caps_are_the_units_own(self) -> None:
        ec, _, _ = open_axis("slider")
        with ec:
            ec.identify()
            self.assertEqual((ec.speed_cap_mm_s, ec.accel_cap_g), (700.0, 1.0))
            self.assertEqual(ec.max_positions(), 3)

    def test_speed_override_on_an_ec(self) -> None:  # UNVERIFIED on an EC
        ec, sim, rec = open_axis("slider")
        with ec:
            st = ec.move_to_position(2, speed_pct=50)
        self.assertEqual(st.position_mm, 75.0)
        self.assertEqual(sim.override_moves, 1)

    def test_ec_reset_still_confirmed_by_pmss(self) -> None:
        ec, sim, _ = open_axis("rod")
        with ec:
            result = ec.software_reset(reset_wait_s=0.0)
        self.assertTrue(result.confirmed)
        self.assertIn("PMSS 1 -> 0", result.confirmed_by)

    def test_ec_named_parameters_in_the_uniform_list(self) -> None:
        ec, _, _ = open_axis("rod")
        with ec:
            params = {p["number"]: p for p in ec.parameters()}
            self.assertEqual(params[2]["item"], "8.9")
            self.assertTrue(params[3]["dangerous"] and params[4]["dangerous"])
            home = ec.write_parameter(4, 1.0, commit=False)  # a home parameter: written with the home warning
            self.assertTrue(any("home parameters" in w for w in home.warnings), home.warnings)
            self.assertEqual(ec.write_parameter("P2", 0.2, commit=False).after, "0.20 mm")


if __name__ == "__main__":
    unittest.main()
