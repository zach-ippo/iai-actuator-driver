"""Parameter ranges and guards, alarm rules, input-range tables and the read-only backup.

CON No.10 (default positioning band) floor and minimum band; per-family CON ranges; EC parameter guards and
categories; CON guards; alarm rules; read-only backup. Simulator and plain values only: no serial port is opened.
"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from iai_actuator_driver import Elecylinder, SimulatedEC
from iai_actuator_driver import alarms as A
from iai_actuator_driver import con_params as C
from iai_actuator_driver import info as I
from iai_actuator_driver import registers as R
from iai_actuator_driver.check import PASS, WARN, run_check
from iai_actuator_driver.ec import IndexCategory
from iai_actuator_driver.errors import SoftLimitError
from iai_actuator_driver.modbus import ModbusRTU

from ec_testutil import FrameRecorder

WRITE_FUNCTIONS = {0x05, 0x06, 0x0F, 0x10, 0x17, 0x45, 0x48}
HISTORY = IndexCategory(R.CATEGORY_ALARM_HISTORY, 0, 3, R.ALARM_HISTORY_ENTRIES, R.ALARM_HISTORY_BASE, 0, 0, 2)
EMPTY_SLOT = [0x0000, 0xFFFF, 0, 0, 0xFFFF, 0xFFFF]


def open_axis(profile: str = "pcon") -> tuple[Elecylinder, SimulatedEC, FrameRecorder]:
    sim = SimulatedEC(profile=profile, time_scale=50.0)
    rec = FrameRecorder()
    ec = Elecylinder(ModbusRTU(sim, frame_logger=rec, min_gap_s=0.0), poll_interval_s=0.005, owns_bus=True)
    return ec, sim, rec


def functions(rec: FrameRecorder) -> set[int]:
    return {frame[1] for frame in rec.tx}


def p(no: int) -> C.ConParam:
    return C.CON_PARAMETERS_BY_NO[no]


def slot(code: int, time_raw: int | None = 1000, detail: int = 0, address: int = 0xFFFF) -> list[int]:
    hi, lo = (0xFFFF, 0xFFFF) if time_raw is None else R.to_u32_words(time_raw)
    return [detail, address, 0, code, hi, lo]


# =========================================================================== CON No.10 default band


class DefaultBandTest(unittest.TestCase):
    def test_floor_is_0_01(self) -> None:
        self.assertEqual(p(10).lo, 0.01)
        with self.assertRaisesRegex(ValueError, "0.01"):
            C.parse(p(10), 0.0)
        self.assertEqual(C.parse(p(10), 0.01), 0.01)
        for family in C.CON_FAMILIES:
            with self.subTest(family=family):
                self.assertEqual(C.for_family(p(10), family).lo, 0.01)
                with self.assertRaises(ValueError):
                    C.parse(p(10), 0.0, family=family)
        self.assertEqual(C.for_family(p(10), "SCON").hi, 9999.99)
        self.assertEqual(C.for_family(p(10), "ACON").hi, 9999.99)
        self.assertEqual(p(10).hi, 999.99)

    def test_minimum_band_formulas(self) -> None:
        band = C.min_positioning_band
        self.assertEqual(band("PCON", 12.0, 800).mm, 0.05)  # 12 / 800 x 3 = 0.045 -> 0.05 (rounded up)
        self.assertEqual(band("PCON", 10.0, 800).mm, 0.04)  # 0.0375 -> 0.04
        self.assertEqual(band("PCON", 16.0, 8192).mm, 0.01)  # RCP6: lead / 8192 (0.002) -> the 0.01 floor
        self.assertEqual(band("PCON", 100.0, 8192).mm, 0.02)  # 0.0122 -> 0.02
        self.assertEqual(band("ACON", 10.0, 800).mm, 0.05)  # ACON x 4: 0.05
        self.assertEqual(band("SCON", 20.0, 131072).mm, 0.01)
        self.assertEqual(band("SCON", 20.0, 800).mm, 0.03)  # x 1: 0.025 -> 0.03
        self.assertEqual(band("PCON", 16.0, 8192, mini_mm=0.03).mm, 0.03)  # MINI wins
        self.assertIsNone(band("PCON", None, None))
        self.assertEqual(band("PCON", None, None, mini_mm=0.02).mm, 0.02)
        guess = band(None, 12.0, None)
        self.assertEqual((guess.mm, guess.certain), (0.05, False))  # 800 pulses assumed: a warning only
        self.assertIn("the PCON rule (the family has no rule of its own in the driver)", guess.doc)
        self.assertNotIn("UNVERIFIED", guess.doc)

    def test_below_the_band_is_refused_with_the_0a1_reason(self) -> None:
        unit = C.ConUnitLimits(lead_mm=12.0, encoder_pulses=800)
        with self.assertRaisesRegex(ValueError, "0x0A1"):
            C.check_value(p(10), 0.04, family="PCON", unit=unit)
        self.assertEqual(C.check_value(p(10), 0.05, family="PCON", unit=unit).value, 0.05)
        # encoder count not known: the 800-pulse estimate only warns
        unsure = C.check_value(p(10), 0.03, family="PCON", unit=C.ConUnitLimits(lead_mm=12.0))
        self.assertTrue(any("0x0A1" in w for w in unsure.warnings))

    def test_unit_limits_from_the_pcon(self) -> None:
        ec, _, _ = open_axis("pcon")
        with ec:
            classes = {ci: I.read_class(ec, ci) for ci in (3, 7, 9)}
            limits = C.con_unit_limits(classes, ec.read_support_words())
        self.assertEqual((limits.lead_mm, limits.encoder_pulses, limits.max_speed, limits.max_accel,
                          limits.max_push_speed, limits.min_band_mm, limits.rotary),
                         (16.0, 8192, 840.0, 1.0, 20.0, 0.01, False))  # fmt: skip
        self.assertEqual(C.min_positioning_band("PCON", limits.lead_mm, limits.encoder_pulses,
                                                mini_mm=limits.min_band_mm).mm, 0.01)  # fmt: skip
        json.dumps(limits.to_dict())

    def test_pcon_write_of_zero_is_refused_before_any_frame(self) -> None:
        ec, _, rec = open_axis("pcon")
        with ec:
            ec.identify()
            rec.clear()
            with self.assertRaises(SoftLimitError):
                ec.write_parameter(10, 0.0)
        self.assertFalse(functions(rec) & WRITE_FUNCTIONS, rec.tx)


# =========================================================================== ranges per family


class FamilyRangeTest(unittest.TestCase):
    # (no, PCON, ACON / DCON, SCON) as documented (PCON = ME0342-4H, ACON = ME0343-3B, SCON = ME0340-9F)
    TABLE = (
        (10, (0.01, 999.99), (0.01, 9999.99), (0.01, 9999.99)),
        (13, (0, 100), (0, 300), (0, 300)),
        (31, (1, 99_999_999), (1, 27661), (1, 99_999_999)),
        (32, (1, 99_999_999), (1, 217270), (1, 99_999_999)),
        (47, (1, 250), (1, 250), (1, 250)),
        (53, (0, 7), (0, 3), (0, 3)),
        (65, (1, 4096), (1, 4096), (1, 99_999_999)),
        (66, (1, 4096), (1, 4096), (1, 99_999_999)),
        (84, (0, 4), (0, 4), (0, 8)),
        (113, (1, 60000), (1, 60000), (1, 1000)),
        (165, (0, 100), (0, 100), (0, 10000)),
    )

    def test_documented_ranges(self) -> None:
        for no, pcon, acon, scon in self.TABLE:
            for family, (lo, hi) in (("PCON", pcon), ("ACON/DCON", acon), ("SCON", scon)):
                with self.subTest(no=no, family=family):
                    q = C.for_family(p(no), family)
                    self.assertEqual((q.lo, q.hi), (lo, hi))

    def test_family_names_and_codes(self) -> None:
        self.assertEqual([C.con_family(x) for x in ("PCON", "acon", "DCON", "ACON/DCON", "SCON")],
                         ["PCON", "ACON/DCON", "ACON/DCON", "ACON/DCON", "SCON"])  # fmt: skip
        self.assertEqual([C.con_family(x) for x in (0xAE, 0xAC, 0xAA)], ["PCON", "ACON/DCON", "SCON"])
        for other in (None, "EC", 0xEC, 0x99, "IAI controller", True):
            self.assertIsNone(C.con_family(other))  # type: ignore[arg-type]

    def test_legal_values_of_one_family_are_illegal_in_another(self) -> None:
        self.assertEqual(C.parse(p(13), 300, family="ACON"), 300)
        with self.assertRaises(ValueError):
            C.parse(p(13), 300, family="PCON")
        self.assertEqual(C.parse(p(113), 1001), 1001)
        with self.assertRaises(ValueError):
            C.parse(p(113), 1001, family="SCON")
        self.assertEqual(C.parse(p(165), 5000, family="SCON"), 5000)
        with self.assertRaises(ValueError):
            C.parse(p(165), 5000)
        with self.assertRaises(ValueError):
            C.parse(p(31), 27662, family="DCON")
        with self.assertRaises(ValueError):
            C.parse(p(65), 4097)
        self.assertEqual(C.parse(p(65), 4097, family="SCON"), 4097)

    def test_an_unknown_family_uses_the_pcon_range_with_a_note(self) -> None:
        result = C.check_value(p(13), 50, family="IAI controller")
        self.assertEqual((result.family, result.hi), (None, 100))
        self.assertIn(C.UNKNOWN_FAMILY_NOTE, result.warnings)
        self.assertIn("the PCON-CB range", C.UNKNOWN_FAMILY_NOTE)
        self.assertNotIn("UNVERIFIED", C.UNKNOWN_FAMILY_NOTE)
        self.assertEqual(C.check_value(p(13), 50, family="PCON").warnings, ())

    def test_wide_percent_encoding(self) -> None:
        q = C.for_family(p(13), "ACON")
        raw = C.encode(q, 0x12340000, 300)
        self.assertEqual(raw, 0x123402FD)  # 300 % = 765 (255 = 100 %), the high word kept
        self.assertEqual(C.decode(q, raw), 300)
        self.assertEqual(C.encode(p(13), 0x12340000, 35), 0x12340059)  # PCON: the low byte only (VERIFIED item)
        self.assertEqual(C.decode(p(13), 0x00000159), 35)

    def test_pcon_refuses_no47_above_250_before_any_frame(self) -> None:
        ec, _, rec = open_axis("pcon")
        with ec:
            ec.identify()
            rec.clear()
            with self.assertRaises(SoftLimitError):
                ec.write_parameter(47, 300)
        self.assertFalse(functions(rec) & WRITE_FUNCTIONS)

    def test_parameter_table(self) -> None:
        for family in (None, "PCON", "ACON", "SCON"):
            rows = {r["number"]: r for r in C.parameter_table(family)}
            json.dumps(rows)
            self.assertEqual(rows[10]["min"], 0.01)
            self.assertEqual(rows[16]["max"], 230400)
            self.assertEqual(rows[88]["category"], "D")
            self.assertTrue(rows[88]["dangerous"])
            self.assertEqual(rows[1]["verified_family"], family is not None)
        self.assertEqual(C.parameter_table("ACON")[[r["number"] for r in C.parameter_table("ACON")].index(13)]["max"],
                         300)  # fmt: skip
        defaults = {r["number"]: r["factory_default"] for r in C.parameter_table("SCON")}
        self.assertEqual((defaults[16], defaults[65], defaults[48]), (38400, 2048, 0.1))


# =========================================================================== CON guards


class ConGuardTest(unittest.TestCase):
    UNIT = C.ConUnitLimits(max_speed=200.0, max_accel=0.7, max_push_speed=20.0)

    def test_baud_only_the_eight_documented_rates(self) -> None:
        self.assertEqual(C.CON_BAUD_RATES, (9600, 14400, 19200, 28800, 38400, 76800, 115200, 230400))
        for rate in C.CON_BAUD_RATES:
            self.assertEqual(C.parse(p(16), rate), rate)
        for rate in (57600, 460800, 1200):
            with self.subTest(rate=rate), self.assertRaises(ValueError):
                C.parse(p(16), rate)
        self.assertEqual(C.decode(p(16), 19), 57600)  # a unit set to 57,600 elsewhere still decodes

    def test_every_d_and_e_parameter_is_dangerous(self) -> None:
        self.assertTrue(p(88).dangerous)
        for q in C.CON_PARAMETERS:
            if C.category(q.no) in ("D", "E"):
                self.assertTrue(q.dangerous, q.no)
        self.assertEqual(C.category(14, "SCON"), "E")
        self.assertEqual(C.category(73, "SCON"), "D")
        self.assertEqual(C.category(14), "")

    def test_unit_maxima_cap_speed_accel_and_push_speed(self) -> None:
        for no, value in ((8, 201), (26, 250), (35, 201), (47, 220), (9, 0.71), (34, 21)):
            with self.subTest(no=no), self.assertRaisesRegex(ValueError, "above"):
                C.check_value(p(no), value, family="PCON", unit=self.UNIT)
        for no, value in ((8, 200), (35, 200), (47, 200), (9, 0.7), (34, 20)):
            with self.subTest(no=no):
                C.check_value(p(no), value, family="PCON", unit=self.UNIT)
        with self.assertRaises(ValueError):
            C.check_value(p(47), 251, family="PCON")  # the 250 cap without a MAXV too
        self.assertEqual(C.check_value(p(8), 5000, family="PCON").value, 5000)  # MAXV not known: the table range

    def test_push_speed_warnings(self) -> None:
        slow = C.check_value(p(34), 4, family="PCON", unit=self.UNIT)
        self.assertEqual(slow.value, 4)
        self.assertTrue(any("5 mm/s" in w for w in slow.warnings))
        self.assertTrue(any("consulted" in w for w in C.check_value(p(34), 10, family="PCON").warnings))

    def test_documented_notes(self) -> None:
        self.assertIn("not 0x9900", p(46).note)
        self.assertTrue(any("0x9900" in w for w in C.check_value(p(46), 50, family="PCON").warnings))
        self.assertIn("PEND", p(39).note)

    def test_audit_and_unit_info_range_check(self) -> None:
        unit = C.ConUnitLimits(lead_mm=12.0, encoder_pulses=800, max_speed=200.0)
        findings = C.audit({10: 0.02, 8: 300.0, 35: 100.0, 5: 1}, family="PCON", unit=unit)
        self.assertEqual([f["number"] for f in findings], [8, 10])
        self.assertIn("0x0A1", findings[1]["problem"])
        ec, _, _ = open_axis("pcon")
        with ec:
            data = ec.unit_info(sections=["identity", "parameters"])
        json.dumps(data)
        check = data["parameters"]["range_check"]
        self.assertEqual((check["family"], check["findings"], check["min_band"]["mm"]), ("PCON", [], 0.01))
        numbered = {row["number"]: row for row in data["parameters"]["numbered"]}
        self.assertEqual((numbered[10]["category"], numbered[10]["family_min"]), ("B", 0.01))
        self.assertEqual((numbered[88]["category"], numbered[88]["dangerous"]), ("D", True))

    def test_ec_check_warns_on_a_band_below_the_minimum(self) -> None:
        ec, _, _ = open_axis("pcon")
        with ec:
            report = run_check(ec, port_label="sim")
            items = {i.name: i for i in report.items}
            self.assertEqual(items["parameter_ranges"].result, PASS, items["parameter_ranges"].detail)
            ec.write_parameter("8.9", 0, commit=False)  # raw item write: No.10 = 0.00 in RAM
            report = run_check(ec, port_label="sim")
        items = {i.name: i for i in report.items}
        self.assertEqual(items["parameter_ranges"].result, WARN)
        self.assertIn("No.10", items["parameter_ranges"].detail)
        self.assertTrue(report.passed)  # a WARN never fails the check


# =========================================================================== EC parameters


class EcParameterTest(unittest.TestCase):
    def test_categories_and_shipping_defaults(self) -> None:
        cats = {q.no: q.category for q in I.USER_PARAMETERS if q.no <= 10}
        self.assertEqual(cats, {1: "A", 2: "A", 3: "C", 4: "B", 5: "A", 6: "A", 7: "A", 8: "A", 9: "A", 10: "A"})
        by_no = I.USER_PARAMETERS_BY_NO
        self.assertEqual((by_no[5].shipping_default, by_no[7].shipping_default, by_no[9].shipping_default),
                         ("Disabled", "Enabled", "Double"))  # fmt: skip
        self.assertTrue(all(q.iaos_only for q in I.USER_PARAMETERS if q.no > 10))
        self.assertFalse(any(q.iaos_only for q in I.USER_PARAMETERS if q.no <= 10))
        self.assertTrue(all("ME3816-6B" in q.doc for q in I.USER_PARAMETERS))
        self.assertIn("9999.69", by_no[1].manual_range)

    def test_manual_names_are_aliases(self) -> None:
        find = I.find_user_parameter
        self.assertEqual(find("Energy-saving setting").no, 8)
        self.assertEqual(find("electromagnetic valve system select").no, 9)
        self.assertEqual(find("Home return direction change").no, 3)
        self.assertEqual(find("LED lighting system automatic switch setting").no, 10)
        self.assertEqual((find("P5").no, find(5).no, find("12").no), (5, 5, 12))
        self.assertIsNone(find("no such parameter"))
        self.assertIsNone(find(True))

    def test_decoded_parameters_carry_the_manual_view(self) -> None:
        ec, _, _ = open_axis("gripper")
        with ec:
            named = {prm["no"]: prm for prm in ec.user_parameters()}
        self.assertEqual((named[4]["category"], named[4]["shipping_default"]), ("B", "per EC specification"))
        self.assertTrue(named[12]["iaos_only"])
        self.assertIn("use as shipped", named[4]["category_text"])

    def test_p4_never_below_the_shipped_value(self) -> None:
        p4 = I.USER_PARAMETERS_BY_NO[4]
        with self.assertRaises(I.ParameterRefused) as cm:
            I.ec_parameter_guard(p4, 0.40, current=0.50)
        self.assertEqual(cm.exception.opt_in, "p4_below_shipped")
        self.assertIn("p.6-10", str(cm.exception))
        with self.assertRaises(I.ParameterRefused):
            I.ec_parameter_guard(p4, 0.60, current=0.70, shipped=0.65)  # the shipped value is the floor
        warnings = I.ec_parameter_guard(p4, 0.60, current=0.50)
        self.assertTrue(any("P1" in w for w in warnings))
        I.ec_parameter_guard(p4, 0.40, current=0.50, allow={"p4_below_shipped"})
        I.ec_parameter_guard(p4, 0.40)  # floor not known: nothing to compare against

    def test_p3_on_a_rotary_needs_its_own_opt_in(self) -> None:
        p3 = I.USER_PARAMETERS_BY_NO[3]
        with self.assertRaises(I.ParameterRefused) as cm:
            I.ec_parameter_guard(p3, 1, rotary=True)
        self.assertEqual(cm.exception.opt_in, "rotary_home_direction")
        self.assertIn("ME3800-5E p.1-36", str(cm.exception))
        self.assertTrue(I.ec_parameter_guard(p3, 1, rotary=True, allow=["rotary_home_direction"]))
        self.assertTrue(any("reverses" in w for w in I.ec_parameter_guard(p3, 1)))

    def test_p9_stays_double_on_an_acr_unit(self) -> None:
        p9 = I.USER_PARAMETERS_BY_NO[9]
        self.assertTrue(I.has_acr_option("EC-S6SAH-100-1-0-ACR"))
        self.assertFalse(I.has_acr_option("EC-S6SACR-100-1-0"))  # clean-room type name, not the option
        self.assertFalse(I.has_acr_option("EC-GRBP8M-20-1-1-PN-WA-WL2"))
        self.assertIsNone(I.has_acr_option(None))
        with self.assertRaises(I.ParameterRefused) as cm:
            I.ec_parameter_guard(p9, 3, model_name="EC-S6SAH-100-1-0-ACR")
        self.assertEqual(cm.exception.opt_in, "acr_single")
        I.ec_parameter_guard(p9, 0, model_name="EC-S6SAH-100-1-0-ACR")
        I.ec_parameter_guard(p9, 4, model_name="EC-S6SAH-100-1-0-ACR-MF")
        I.ec_parameter_guard(p9, 3, model_name="EC-S6SACR-100-1-0")

    def test_smooth_accel_rules(self) -> None:
        p5 = I.USER_PARAMETERS_BY_NO[5]
        self.assertTrue(any("2 s" in w for w in I.ec_parameter_guard(p5, 1)))
        self.assertEqual(I.ec_parameter_guard(p5, 0), [])
        slow = I.smooth_accel_check(100.0, 0.003, smooth_enabled=True)  # 100 / (0.003 x 9806.65) = 3.4 s
        self.assertTrue(slow["over_limit"])
        self.assertAlmostEqual(slow["accel_time_s"], 3.399, places=3)
        fast = I.smooth_accel_check(100.0, 0.3, 0.1, smooth_enabled=True)
        self.assertEqual((fast["over_limit"], fast["warnings"]), (False, []))
        self.assertFalse(I.smooth_accel_check(100.0, 0.003, smooth_enabled=False)["over_limit"])
        retarget = I.smooth_accel_check(100.0, 0.3, smooth_enabled=True, retarget=True)
        self.assertTrue(any("overload" in w for w in retarget["warnings"]))
        self.assertEqual(I.ramp_time_s(100.0, 0.0), float("inf"))

    def test_iaos_only_and_p10_warnings(self) -> None:
        self.assertTrue(any("IA-OS" in w for w in I.ec_parameter_guard(I.USER_PARAMETERS_BY_NO[12], 1)))
        self.assertTrue(any("p.6-16" in w for w in I.ec_parameter_guard(I.USER_PARAMETERS_BY_NO[10], 1)))

    def test_ec_parameter_table(self) -> None:
        rows = {r["no"]: r for r in I.ec_parameter_table()}
        json.dumps(rows)
        self.assertEqual(sorted(rows), [*range(1, 19), 143])
        self.assertEqual(rows[2]["driver_range"], {"min": 0.01, "max": 1.0})
        self.assertTrue(rows[3]["home_parameter"] and rows[4]["home_parameter"])
        self.assertEqual(rows[143]["manual_range"], "50-99 %, 100 = off")


# =========================================================================== input ranges


class InputRangesTest(unittest.TestCase):
    def test_every_product_line(self) -> None:
        tables = I.input_ranges()
        json.dumps(tables)
        self.assertEqual(list(tables), list(I.PRODUCT_LINES))
        for line, table in tables.items():
            with self.subTest(line=line):
                self.assertTrue(table["title"] and table["rule"] and table["moves"])
                self.assertTrue(all(row["doc"] for row in table["moves"] + table["other"]))
        self.assertEqual(len(tables["EC"]["parameters"]), 19)
        acon = I.input_ranges("CON-CB", family="ACON")
        self.assertEqual(acon["family"], "ACON/DCON")
        self.assertEqual(next(r for r in acon["parameters"] if r["number"] == 13)["max"], 300)
        with self.assertRaises(ValueError):
            I.input_ranges("RCON-EC")

    def test_product_line(self) -> None:
        self.assertEqual(I.product_line("EC"), "EC")
        self.assertEqual(I.product_line("PCON"), "CON-CB")
        self.assertEqual(I.product_line("IAI controller"), "CON-CB")
        self.assertEqual(I.product_line("PCON", gateway="rcon"), "RCON")
        self.assertEqual(I.product_line(None, gateway="robonet-gateway"), "ROBONET")


# =========================================================================== alarms


class AlarmFamilyRulesTest(unittest.TestCase):
    def test_fieldbus_link_error_is_operation_cancelled(self) -> None:
        self.assertEqual(A.alarm_level(0x0F1), A.LEVEL_OPERATION_CANCELLED)
        self.assertIn("ME0342-4H p.8-21", A.alarm_info(0x0F1)["remedy"])

    def test_con_message_codes(self) -> None:
        for code in (0x02C, 0x02D, 0x02E):
            with self.subTest(code=hex(code)):
                self.assertEqual(A.alarm_level(code), A.LEVEL_UNKNOWN)  # an EC: fail safe
                self.assertEqual(A.alarm_level(code, system_code=0xAE), A.LEVEL_MESSAGE)
                self.assertEqual(A.alarm_level(code, family="SCON"), A.LEVEL_MESSAGE)
                self.assertFalse(A.alarm_is_fault(code, system_code=0xAC))
                self.assertTrue(A.alarm_is_fault(code))
                self.assertNotEqual(A.alarm_name(code), f"alarm {code:#05x}")
                info = A.alarm_info(code, family="PCON")
                self.assertTrue(info["warning"] and not info["fault"])
                self.assertEqual(info["family"], "PCON")
        self.assertIn("No.111", A.alarm_info(0x02E, family="PCON")["remedy"])

    def test_family_clearing_rules_keep_the_conservative_level(self) -> None:
        for code in (0x0CE, 0x0D2):
            for family in ("PCON", "ACON", "SCON", None):
                self.assertEqual(A.alarm_info(code, family=family)["level"], A.LEVEL_COLD_START)
        self.assertIn("alarm reset is enough", A.alarm_info(0x0CE, family="PCON")["remedy"])
        self.assertIn("cold start", A.alarm_info(0x0CE, family="SCON")["remedy"])
        self.assertIn("SCON-CB", A.alarm_info(0x0CE)["remedy"])  # no family: both rules named
        self.assertIn("ME0340-9F p.8-23", A.alarm_info(0x0EE, absolute_encoder=True, family="SCON")["remedy"])
        self.assertEqual(A.alarm_info(0x0EE, absolute_encoder=True, family="PCON")["level"],
                         A.LEVEL_OPERATION_CANCELLED)  # fmt: skip

    def test_abs_error_detail_codes(self) -> None:
        for detail, word in ((0x0001, "battery"), (0x0002, "cable"), (0x0006, "absolute reset"),
                             (0x000B, "motor replaced")):  # fmt: skip
            self.assertIn(word, A.describe_alarm_detail(0x0EE, detail, 0xFFFF))
        self.assertEqual(A.describe_alarm_detail(0x0EE, 0x0003, 0xFFFF), "")
        self.assertIn("absolute reset", A.alarm_info(0x0EE, absolute_encoder=True)["remedy"])

    def test_documented_remedies(self) -> None:
        self.assertIn("No.10", A.alarm_info(0x0A1)["remedy"])
        self.assertIn("5 s", A.alarm_info(0x05C)["remedy"])
        self.assertIn("No.111", A.alarm_info(0x069)["remedy"])
        self.assertIn("soft limit", A.alarm_info(0x0A7)["remedy"])
        self.assertIn("empty row", A.alarm_info(0x0A2)["remedy"])

    def test_groups_and_warning_clearing(self) -> None:
        self.assertIn("A-E", A.GROUP_NAMES["F"])
        self.assertIn("target", A.alarm_info(0x04E)["reset"])  # maintenance warning 1 on an EC
        self.assertIn("alarm reset", A.alarm_info(0x048)["reset"])
        self.assertEqual(A.alarm_info(0x04E, family="PCON")["reset"], "alarm reset (the axis keeps running meanwhile)")

    def test_time_with_and_without_the_calendar(self) -> None:
        self.assertEqual(A.rtc_datetime(0).isoformat(), "2000-01-01T00:00:00")
        rtc = A.alarm_time(86400 + 3661, rtc_in_use=True)
        self.assertEqual((rtc["clock"], rtc["occurred_at"], rtc["time_after_power_on"]),
                         ("rtc", "2000-01-02 01:01:01", None))  # fmt: skip
        plain = A.alarm_time(90061001, rtc_in_use=False)
        self.assertEqual((plain["clock"], plain["occurred_at"], plain["time_after_power_on"]),
                         ("power_on", None, "1 d 01:01:01.001"))  # fmt: skip
        self.assertEqual(A.alarm_time(None, rtc_in_use=True)["clock"], None)

    def test_unit_info_history_with_the_calendar(self) -> None:
        stamp = 26 * 365 * 86400  # about 2025-12-25, in seconds since 2000-01-01
        words = slot(0x0D9, stamp, detail=2) + EMPTY_SLOT * 31
        alarms = I.decode_alarm_history(HISTORY, words, 0, 0x3098, 0, None, rtc_in_use=True, system_code=0xAE)
        entry = alarms["history"][0]
        self.assertEqual((entry["clock"], entry["occurred_at"][:4], entry["time_after_power_on"]),
                         ("rtc", "2025", None))  # fmt: skip
        self.assertTrue(alarms["rtc_in_use"])
        plain = I.decode_alarm_history(HISTORY, words, 0, 0x3098, 0)
        self.assertEqual((plain["history"][0]["clock"], plain["history"][0]["occurred_at"]), ("power_on", None))

    def test_timn_on_a_pcon(self) -> None:
        ec, _, _ = open_axis("pcon")
        with ec:
            block = I.read_class(ec, R.CATEGORY_MAINTENANCE)
            data = ec.unit_info(sections=["identity", "alarms", "maintenance"])
        on = I.decode_maintenance(block, rtc=True, family="PCON")
        self.assertEqual(on["calendar_time"], "2000-01-01 00:01:39")  # 0x63 = 99 s at 0x8420
        self.assertIsNone(I.decode_maintenance(block, rtc=False, family="PCON")["calendar_time"])
        # an SCON keeps TIMN at 0x841E (the PCON block holds 0xBDA = 3034 there): the address follows the family
        self.assertEqual(I.decode_maintenance(block, rtc=True, family="SCON")["calendar_time"], "2000-01-01 00:50:34")
        self.assertIsNone(I.decode_maintenance(block, rtc=True, family="EC")["calendar_time"])  # no TIMN on an EC
        self.assertFalse(data["alarms"]["rtc_in_use"])  # the simulated PCON's calendar is off (SSSE b8 = 0)
        self.assertTrue(all(h["clock"] in ("power_on", None) for h in data["alarms"]["history"]))


# =========================================================================== backup


class BackupTest(unittest.TestCase):
    def test_every_profile_round_trips_read_only(self) -> None:
        for profile in ("slider", "gripper", "rod", "pcon", "rotary", "unknown"):
            with self.subTest(profile=profile):
                ec, _, rec = open_axis(profile)
                with ec:
                    data = I.backup(ec)
                self.assertFalse(functions(rec) & WRITE_FUNCTIONS)
                self.assertEqual(json.loads(json.dumps(data)), data)
                self.assertEqual(data["errors"], [])
                self.assertEqual(data["meta"]["schema"], I.BACKUP_SCHEMA)
                self.assertTrue(data["meta"]["read_only"])
                self.assertTrue(data["directory"]["records"])
                self.assertTrue(data["parameters"]["classes"] and data["parameters"]["named"])
                self.assertTrue(all("address" in it and "hex" in it
                                    for c in data["parameters"]["classes"] for it in c["items"]))  # fmt: skip
                self.assertTrue(data["positions"]["rows"])
                self.assertIsNotNone(data["alarms"])
                self.assertIsNotNone(data["maintenance"])

    def test_rows_used_all_and_listed(self) -> None:
        ec, _, _ = open_axis("pcon")
        with ec:
            used = I.backup(ec)["positions"]
            listed = I.backup(ec, rows=[5, 1, 2])["positions"]
            everything = I.backup(ec, rows="all")["positions"]
            bad = I.backup(ec, rows=[99999])
        self.assertEqual([r["n"] for r in used["rows"]], list(range(64)))
        self.assertEqual([r["n"] for r in listed["rows"]], [1, 2, 5])
        self.assertEqual(len(everything["rows"]), everything["rows_in_table"])
        self.assertTrue(any(e.startswith("positions: ValueError") for e in bad["errors"]))
        row = used["rows"][1]
        self.assertEqual(len(row["words"]), 16)
        self.assertIn("position_mm", row["decoded"])
        ec, _, _ = open_axis("slider")
        with ec:
            rows = [r["n"] for r in I.backup(ec)["positions"]["rows"]]
        self.assertEqual(rows, [0, 1, 2, *R.POSITION_SIMPLE_DATA_ROWS])

    def test_backup_files(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            ec, _, _ = open_axis("gripper")
            with ec:
                data = I.backup(ec, Path(tmp) / "gripper.json")
            saved = json.loads((Path(tmp) / "gripper.json").read_text(encoding="utf-8"))
            self.assertEqual(saved["identity"], data["identity"])
            self.assertEqual(saved["meta"]["path"], str(Path(tmp) / "gripper.json"))
            ec1, _, _ = open_axis("rod")
            ec2, _, _ = open_axis("pcon")
            with ec1, ec2:
                paths = I.backup_axes({"rod/1": ec1, "pcon": ec2}, Path(tmp) / "cell")
            self.assertEqual(set(paths), {"rod/1", "pcon"})
            self.assertTrue(paths["rod/1"].endswith("rod_1.json"))
            for path in paths.values():
                self.assertEqual(json.loads(Path(path).read_text(encoding="utf-8"))["errors"], [])


if __name__ == "__main__":
    unittest.main()
