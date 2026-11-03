"""Alarm rules: the fail-safe level of unlisted message-range codes, the 0x0EE level by encoder type, 0x0CE /
0x0D2 as cold start, and the alarm history ending at the first empty slot.

Everything runs against the simulator or on plain words (no serial port is opened).
"""

from __future__ import annotations

import unittest

from iai_actuator_driver import Elecylinder, SimulatedEC
from iai_actuator_driver import alarms as A
from iai_actuator_driver import info as I
from iai_actuator_driver import registers as R
from iai_actuator_driver.ec import IndexCategory, Status
from iai_actuator_driver.modbus import ModbusRTU

EMPTY_SLOT = [0x0000, 0xFFFF, 0, 0, 0xFFFF, 0xFFFF]
HISTORY = IndexCategory(R.CATEGORY_ALARM_HISTORY, 0, 3, R.ALARM_HISTORY_ENTRIES, R.ALARM_HISTORY_BASE, 0, 0, 2)


def open_ec(profile: str) -> tuple[Elecylinder, SimulatedEC]:
    sim = SimulatedEC(profile=profile, time_scale=50.0)
    return Elecylinder(ModbusRTU(sim, min_gap_s=0.0), poll_interval_s=0.005, owns_bus=True), sim


def slot(code: int, time_ms: int | None = 1000, detail: int = 0, address: int = 0xFFFF) -> list[int]:
    hi, lo = (0xFFFF, 0xFFFF) if time_ms is None else R.to_u32_words(time_ms)
    return [detail, address, 0, code, hi, lo]


def status(code: int, dss1: int = 0x3098) -> Status:
    return Status.from_registers([0, 7395, code, 0, 0, dss1, 0x8000, 0x33C2])


class MessageLevelTest(unittest.TestCase):
    def test_message_codes_are_the_named_codes_of_the_range(self) -> None:
        named = {code for code in A.CONTROLLER_ALARM_NAMES if 0x011 <= code <= 0x07F}
        self.assertEqual(A.MESSAGE_CODES, named)
        self.assertTrue({0x047, 0x048, 0x049, 0x04A, 0x04E, 0x04F} <= A.MESSAGE_CODES)

    def test_listed_codes_are_warnings(self) -> None:
        for code in sorted(A.MESSAGE_CODES):
            with self.subTest(code=hex(code)):
                self.assertEqual(A.alarm_level(code), A.LEVEL_MESSAGE)
                self.assertFalse(A.alarm_is_fault(code))
                self.assertFalse(A.alarm_is_fault(code, warning_bit=True))
                self.assertTrue(A.alarm_is_fault(code, alarm_bit=True))  # ALMH always makes it a fault
                info = A.alarm_info(code)
                self.assertTrue(info["warning"] and not info["fault"])

    def test_unlisted_codes_of_the_range_are_faults_without_alml(self) -> None:
        unlisted = [code for code in range(0x011, 0x080) if code not in A.MESSAGE_CODES]
        self.assertIn(0x011, unlisted)
        self.assertIn(0x07D, unlisted)
        for code in unlisted:
            with self.subTest(code=hex(code)):
                self.assertEqual(A.alarm_level(code), A.LEVEL_UNKNOWN)
                self.assertTrue(A.alarm_is_fault(code))  # fail safe
                self.assertFalse(A.alarm_is_fault(code, warning_bit=True))  # the EC flags it as a warning
                self.assertTrue(A.alarm_is_fault(code, alarm_bit=True, warning_bit=True))
                info = A.alarm_info(code)
                self.assertTrue(info["fault"] and not info["warning"])
                self.assertIn("ALML", info["remedy"])

    def test_status_follows_the_rule(self) -> None:
        unlisted = status(0x042)
        self.assertEqual((unlisted.alarm, unlisted.warning, unlisted.alarm_level), (True, False, "unknown"))
        self.assertFalse(unlisted.ready)
        flagged = status(0x042, 0x3098 | R.DSS1_ALML)
        self.assertEqual((flagged.alarm, flagged.warning), (False, True))
        listed = status(0x04E)
        self.assertEqual((listed.alarm, listed.warning, listed.alarm_level), (False, True, "message"))

    def test_unit_info_lamps(self) -> None:
        live = I.decode_monitor({"ALMC": 0x042, "DSS1": 0x3098})
        self.assertEqual((live["io"]["lamps"]["alarm"], live["io"]["lamps"]["warning"]), (True, False))
        live = I.decode_monitor({"ALMC": 0x04E, "DSS1": 0x3098})
        self.assertEqual((live["io"]["lamps"]["alarm"], live["io"]["lamps"]["warning"]), (False, True))


class AbsErrorLevelTest(unittest.TestCase):
    def test_encoder_type_from_the_support_words(self) -> None:
        f = A.absolute_encoder_from_support
        self.assertTrue(f({"SUPT": 0x60002BFF, "SUP2": 0x838C009A}))  # WA slider, read on hardware
        self.assertTrue(f({"SUPT": 0x60002BD7, "SUP2": 0x4384009A}))  # WA gripper, read on hardware
        self.assertTrue(f({"SUPT": 0x60000BD7, "SUP2": 0x00040000}))  # MABS alone
        self.assertTrue(f({"SUP2": 0x00040000}))
        self.assertFalse(f({"SUPT": 0x60000BD7, "SUP2": 0x4300009A}))
        self.assertFalse(f({"SUPT": 0x60000BD7}))
        self.assertIsNone(f({"SUP2": 0x4300009A}))  # no SUPT: ABSE unknown
        self.assertIsNone(f({}))
        self.assertIsNone(f(None))

    def test_levels_and_texts(self) -> None:
        absolute = A.alarm_info(0x0EE, absolute_encoder=True)
        self.assertEqual(absolute["level"], A.LEVEL_OPERATION_CANCELLED)
        self.assertEqual(absolute["home_needed"], "home lost: alarm reset, then a home return")
        incremental = A.alarm_info(0x0EE, absolute_encoder=False)
        self.assertEqual(incremental["level"], A.LEVEL_COLD_START)
        self.assertIn("software reset", incremental["home_needed"])
        self.assertIn("software reset", incremental["reset"])
        unknown = A.alarm_info(0x0EE)
        self.assertEqual(unknown["level_text"],
                         "operation cancelled on absolute units / cold start on incremental units")
        self.assertIn("absolute units: alarm reset", unknown["reset"])
        self.assertIn("incremental units: software reset", unknown["reset"])
        self.assertIn("home return", unknown["home_needed"])
        self.assertTrue(all(i["fault"] for i in (absolute, incremental, unknown)))

    def test_unit_info_passes_the_encoder_type(self) -> None:
        for profile, absolute, level in (("gripper", True, "operation_cancelled"), ("rod", False, "cold_start")):
            with self.subTest(profile=profile):
                ec, sim = open_ec(profile)
                with ec:
                    sim.trigger_alarm(0x0EE)
                    data = ec.unit_info(sections=["identity", "alarms"])
                self.assertEqual(data["identity"]["capabilities"]["support"]["flags"]["absolute_encoder"], absolute)
                alarms = data["alarms"]
                self.assertEqual(alarms["absolute_encoder"], absolute)
                self.assertEqual((alarms["current"]["code"], alarms["current"]["level"]), (0x0EE, level))
                self.assertEqual((alarms["history"][0]["code"], alarms["history"][0]["level"]), (0x0EE, level))
                # the alarms section alone reads the support words too
                ec2, sim2 = open_ec(profile)
                with ec2:
                    sim2.trigger_alarm(0x0EE)
                    only = ec2.unit_info(sections=["alarms"])["alarms"]
                self.assertEqual((only["absolute_encoder"], only["current"]["level"]), (absolute, level))


class SupplyVoltageLevelTest(unittest.TestCase):
    def test_cold_start_on_every_unit(self) -> None:
        for code in (0x0CE, 0x0D2):
            for system_code in (0xEC, 0xED):
                with self.subTest(code=hex(code), system_code=hex(system_code)):
                    self.assertEqual(A.alarm_level(code, system_code=system_code), A.LEVEL_COLD_START)
            info = A.alarm_info(code)
            self.assertEqual((info["level"], info["fault"], info["group"]), (A.LEVEL_COLD_START, True, "E"))
            self.assertEqual(info["home_needed"], "incremental units: home return after the reset")
            self.assertIn("24 V supply", info["remedy"])


class HistoryEndTest(unittest.TestCase):
    def test_slots_stop_at_the_first_empty_one(self) -> None:
        words = slot(0x0C1, 500) + slot(R.ALARM_POWER_UP, None) + EMPTY_SLOT + slot(0x0EE, 300) + EMPTY_SLOT * 28
        self.assertEqual(A.alarm_history_slots(words),
                         [(0, 0x0C1, 0, 0xFFFF, 500), (1, R.ALARM_POWER_UP, 0, 0xFFFF, None)])
        every = A.alarm_history_slots(words, include_empty=True)
        self.assertEqual(len(every), 32)
        self.assertEqual(every[3][:2], (3, 0x0EE))
        self.assertEqual(A.alarm_history_slots(EMPTY_SLOT * 32), [])

    def test_unit_info_history_ends_at_the_first_empty_slot(self) -> None:
        words = slot(0x0C1, 500) + slot(R.ALARM_POWER_UP, None) + EMPTY_SLOT + slot(0x0EE, 300) + EMPTY_SLOT * 28
        alarms = I.decode_alarm_history(HISTORY, words, 0, 0x3098, 0)
        self.assertEqual([h["code"] for h in alarms["history"]], [0x0C1])
        counts = (alarms["slots"], alarms["alarm_count"], alarms["power_up_markers"], alarms["empty_slots"])
        self.assertEqual(counts, (32, 1, 1, 30))
        self.assertIn("first empty slot", alarms["notes"]["order"])


if __name__ == "__main__":
    unittest.main()
