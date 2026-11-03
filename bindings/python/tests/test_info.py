"""Unit info (iai_actuator_driver.info): bit decoders, parameter decoding, alarm names, read_class / read_block bounds,
Elecylinder.unit_info() on both simulator profiles, ``iai-actuator-driver info`` and the iai-check report.

Everything runs against the simulator; the parameter expectations are the real baselines read from the units
(gripper: the values IA-OS's "User parameter edit" screen showed, VERIFIED).
"""

from __future__ import annotations

import json
import struct
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from iai_actuator_driver import Elecylinder, SimulatedEC
from iai_actuator_driver import info as I
from iai_actuator_driver import registers as R
from iai_actuator_driver.alarms import CONTROLLER_ALARM_NAMES, alarm_name
from iai_actuator_driver.ec import AlarmRecord
from iai_actuator_driver.errors import ModbusTimeout, UnsafeCommandError
from iai_actuator_driver.modbus import ModbusRTU

from test_cli import SimFactory, run

READ_ONLY_FUNCTIONS = {0x03, 0x11}


def open_ec(profile: str = "slider", **sim_kwargs: object) -> tuple[Elecylinder, SimulatedEC]:
    sim = SimulatedEC(profile=profile, time_scale=50.0, **sim_kwargs)  # type: ignore[arg-type]
    ec = Elecylinder(ModbusRTU(sim, min_gap_s=0.0), poll_interval_s=0.005, owns_bus=True)
    return ec, sim


def fc03_ranges(sim: SimulatedEC) -> list[tuple[int, int]]:
    """(start, count) of every FC03 request the simulator received."""
    out = []
    for frame in sim.requests:
        if frame[1] == 0x03:
            start, count = struct.unpack(">HH", frame[2:6])
            out.append((start, count))
    return out


def assert_reads_inside_directory(test: unittest.TestCase, ec: Elecylinder, sim: SimulatedEC) -> None:
    """Every FC03 the simulator saw lies inside one directory block's certain extent, an ID map or the directory."""
    header, records = I.read_directory(ec)
    allowed = [(0, 8 + 8 * header.record_count)]
    for rec in records:
        allowed.append((rec.data_address, rec.data_address + I.block_extent(rec)[0]))
        if rec.id_list_address:
            allowed.append((rec.id_list_address, rec.id_list_address + 2 * rec.member_count))
    ranges = fc03_ranges(sim)
    test.assertGreater(len(ranges), 20)
    for start, count in ranges:
        test.assertTrue(
            any(lo <= start and start + count <= hi for lo, hi in allowed),
            f"FC03 0x{start:04X} x{count} is outside every directory block",
        )


def item(class_id: int, item_id: int, raw: int, size: int = 2, address: int = 0x0800) -> I.ClassItem:
    return I.ClassItem(class_id, item_id, 0, 0, address, size, raw)


def block(class_id: int, *items: I.ClassItem) -> I.ClassBlock:
    return I.ClassBlock(class_id, 0x0800, 2 * len(items), 2, 1, 0x5200, tuple(items))


# =========================================================================== decoders


class DecodeBitsTest(unittest.TestCase):
    def test_dss1_after_a_move(self) -> None:
        word = I.decode_bits(0x7099, I.DSS1_BITS)  # live: idle after a normal move, SFTY on
        self.assertEqual(word["on"], ["SFTY", "PWR", "SV", "BKRL", "HEND", "PEND", "MEND"])
        self.assertEqual(word["hex"], "0x7099")
        self.assertEqual([b["bit"] for b in word["bits"]][:3], [15, 14, 13])  # high bit first
        pend = next(b for b in word["bits"] if b["name"] == "PEND")
        self.assertTrue(pend["on"] and pend["confirmed"])
        self.assertEqual(pend["evidence"], "VERIFIED-LIVE")
        emgs = next(b for b in word["bits"] if b["name"] == "EMGS")
        self.assertFalse(emgs["on"] or emgs["confirmed"])
        self.assertEqual(word["unknown_set"], [])

    def test_table_by_name_and_unknown_bits(self) -> None:
        word = I.decode_bits(0x4043, "ZONS")  # b6 is not named in the ZONS table
        self.assertEqual(word["on"], ["LS2", "Z2", "Z1", "b6"])
        self.assertEqual(word["unknown_set"], [6])
        self.assertEqual(word["bits"][-1]["evidence"], "not named")
        with self.assertRaises(ValueError):
            I.decode_bits(0x10000, "DSS1")
        with self.assertRaises(KeyError):
            I.decode_bits(0, "NOPE")

    def test_dsse_push_contact_and_stat_32_bit(self) -> None:
        self.assertIn("PHDT", I.decode_bits(0x37C3, "DSSE")["on"])  # live: holding after contact (EC-DS4H slider)
        self.assertIn("PMSS", I.decode_bits(0x37C3, "DSSE")["on"])
        stat = I.decode_bits(0x0008801F, I.STAT_BITS, width=32)  # live gripper STAT
        self.assertEqual(stat["hex"], "0x0008801F")
        self.assertEqual(stat["on"], ["SFD", "CARS", "RMDS", "HEND", "SV", "SON", "MPOW"])
        self.assertTrue(all(not b["confirmed"] for b in stat["bits"]))  # names only

    def test_dopm_layouts(self) -> None:
        three = I.io_bit_tables("3pos")
        two = I.io_bit_tables("2pos")
        self.assertEqual(I.decode_bits(0x000C, three["DOPM"])["on"], ["*ALM", "LS2"])  # slider at 75.00 mm
        self.assertEqual(I.decode_bits(0x0009, three["DOPM"])["on"], ["*ALM", "LS0"])  # slider at 0.00 mm
        self.assertEqual(I.decode_bits(0x0006, two["DOPM"])["on"], ["*ALM", "LS1"])  # gripper at 9.50 mm
        self.assertEqual(I.decode_bits(0x0005, two["DOPM"])["on"], ["*ALM", "LS0"])  # gripper open
        alm = next(b for b in I.decode_bits(0x0004, two["DOPM"])["bits"] if b["name"] == "*ALM")
        self.assertTrue(alm["active_low"] and alm["on"])  # 1 = no alarm
        self.assertEqual(I.decode_bits(0x0004, three["DOPM"])["on"], ["LS2"])  # same word, other layout
        for table in (three["DIPM"], I.DIPM_BITS_SINGLE):
            self.assertTrue(all(not b.confirmed for b in table))  # no PIO wired on the slider: unconfirmed
        # the gripper's inputs pressed by hand (2026-10-30): ST0 = bit 0 and ST1 = bit 1 seen, RES (bit 2) not yet
        self.assertEqual([(b.name, b.confirmed) for b in two["DIPM"]], [("RES", False), ("ST1", True), ("ST0", True)])
        self.assertTrue(all(b.evidence.startswith("VERIFIED-LIVE") for b in two["DIPM"] if b.confirmed))
        self.assertIn("not yet seen live", two["DIPM"][0].evidence)
        self.assertEqual([b.name for b in three["DIPM"]], ["RES", "ST1", "ST0"])
        self.assertEqual([b.name for b in two["DIPM"]], ["RES", "ST1", "ST0"])
        self.assertEqual(three["DIPM"][0].bit, 3)  # 3-position: RES moves to IN3
        self.assertEqual(two["DIPM"][0].bit, 2)

    def test_position_mode(self) -> None:
        mode = I.position_mode(4)
        self.assertEqual(
            (mode["name"], mode["positions"], mode["layout"], mode["confirmed"]), ("3-position", 3, "3pos", False)
        )
        mode = I.position_mode(0)
        self.assertEqual((mode["layout"], mode["confirmed"]), ("2pos", True))
        self.assertIn("double", mode["name"])
        single = I.position_mode(3)
        self.assertIn("single", single["name"])
        self.assertIs(I.io_bit_tables(single)["DIPM"], I.DIPM_BITS_SINGLE)
        self.assertEqual(I.position_mode(None, "MWB-EC-DSRR4-WA-MF")["layout"], "3pos")  # model option MF
        self.assertEqual(I.position_mode(None, "EC-GRBP8M-20-1-1-PN-WA-WL2")["source"], "assumed")
        self.assertIn("unknown user parameter 9 value 7", I.position_mode(7)["note"])

    def test_category_extent(self) -> None:
        self.assertEqual(R.category_extent(0x0007, 29, 1, 2), (58, True, 58))
        self.assertEqual(R.category_extent(0x0005, 3, 32, 2), (192, True, 192))
        self.assertEqual(R.category_extent(0x0101, 22, 1, 0), (34, True, 34))  # monitor 0x9000-0x9021
        self.assertEqual(R.category_extent(0x0103, 7, 1, 0), (10, True, 10))  # 0x9900-0x9909
        # compact monitor 0x9F00-0x9F27 (VERIFIED, read live): 9F18 OLLV .. 9F1C-1D ODOM
        self.assertEqual(R.category_extent(0x0114, 20, 1, 0), (40, True, 40))
        self.assertEqual(R.category_extent(0x0112, 18, 1, 0), (18, False, 36))  # unknown sizes: lower bound
        self.assertEqual(R.category_extent(0x0101, 20, 1, 0), (20, False, 40))  # another DQ: not the known size


# =========================================================================== user parameters


class ParameterDecodingTest(unittest.TestCase):
    def decode(self, no: int, raw: int) -> dict:  # type: ignore[type-arg]
        p = next(p for p in I.USER_PARAMETERS if p.no == no)
        return I.decode_user_parameters({p.class_id: block(p.class_id, item(p.class_id, p.item_id, raw))})[
            [q.no for q in I.USER_PARAMETERS].index(no)
        ]

    def test_encodings_from_the_one_at_a_time_diffs(self) -> None:
        # one parameter changed at a time on the gripper (VERIFIED): baseline raw -> test value raw
        self.assertEqual(self.decode(1, 980)["text"], "9.50 mm")
        self.assertEqual(self.decode(1, 970)["value"], 9.40)
        self.assertEqual(self.decode(2, 20)["text"], "0.20 mm")
        self.assertEqual((self.decode(3, 0x7)["text"], self.decode(3, 0x7)["value"]), ("1:Forward", 1))
        self.assertEqual((self.decode(3, 0x8)["text"], self.decode(3, 0x8)["value"]), ("0:Reverse", 0))
        self.assertEqual(self.decode(4, 60)["text"], "0.60 mm")
        self.assertEqual(self.decode(5, 0x440)["text"], "1:Enable")  # P5 set, P6 set
        self.assertEqual(self.decode(6, 0x040)["text"], "0:Disable")
        self.assertEqual(self.decode(7, 0x00808008)["text"], "1:Disable")  # inverted sense
        self.assertEqual(self.decode(7, 0x00008008)["text"], "0:Enable")
        self.assertEqual(self.decode(9, 3)["text"], "3:Single")
        self.assertEqual(self.decode(9, 9)["text"], "9: unknown")
        self.assertEqual(self.decode(10, 0x00048008)["text"], "1:Enable")
        self.assertEqual(self.decode(12, 0x20410000)["text"], "1:Instantaneous energization")

    def test_p3_is_the_parity_of_the_home_pattern(self) -> None:
        # ORG item 9.16 is a home pattern number; its parity is the direction (odd = 1:Forward). Real values
        # (VERIFIED): gripper 7 = 1:Forward, 8 = 0:Reverse (as IA-OS shows them), rod 7 = 1:Forward.
        for raw, value in ((7, 1), (8, 0), (3, 1), (4, 0), (9, 1), (10, 0), (1, 1), (0x00010007, 1)):
            with self.subTest(raw=raw):
                decoded = self.decode(3, raw)
                self.assertEqual((decoded["value"], decoded["pattern"]), (value, raw))
                self.assertEqual(decoded["text"], {1: "1:Forward", 0: "0:Reverse"}[value])
        for raw in (0, 0xFFFFFFFF):  # P3 not settable (UNVERIFIED)
            with self.subTest(raw=raw):
                decoded = self.decode(3, raw)
                self.assertIsNone(decoded["value"])
                self.assertIn("not settable", decoded["text"])
        p3 = I.USER_PARAMETERS_BY_NO[3]
        # write = pattern -/+ 1, kept when the direction already matches
        for raw, number, new in ((8, 1, 7), (7, 0, 8), (4, 1, 3), (3, 0, 4), (10, 1, 9), (9, 0, 10), (7, 1, 7),
                                 (8, 0, 8)):
            with self.subTest(raw=raw, number=number):
                self.assertEqual(I.encode_user_parameter(p3, raw, number), new)
        for raw in (0, 0xFFFFFFFF):
            with self.assertRaises(ValueError):
                I.encode_user_parameter(p3, raw, 1)
        self.assertEqual(I.parse_parameter_value(p3, "forward"), 1)
        self.assertEqual(I.parse_parameter_value(p3, "0:Reverse"), 0)

    def test_located_extra_parameters_are_read_only(self) -> None:
        self.assertEqual(I.READ_ONLY_PARAMETERS, {8, 11, 13, 14, 15, 16, 17, 18})
        self.assertEqual(self.decode(8, 0x05350000)["text"], "1:Disable")  # rod ATYP: b18 = 1 (as IA-OS shows it)
        self.assertEqual(self.decode(8, 0x00080000)["text"], "0:Enable")  # gripper ATYP: b18 = 0
        self.assertEqual(self.decode(13, 0x0901048C)["text"], "1:Enable")  # FSTP b10
        self.assertEqual(self.decode(16, 0x00008408)["text"], "1:Enable")  # SPIO b10
        p11 = I.USER_PARAMETERS_BY_NO[11]
        fi = I.ClassItem(8, 25, 0, 0, 0x0830, 2, 0x18191A10)  # slider FI00 byte 1 = 26
        fo = I.ClassItem(8, 29, 0, 0, 0x0832, 2, 0x292A2B10)  # slider FO00 byte 1 = 43
        decoded = I.decode_user_parameter(p11, fi, fo)
        self.assertEqual((decoded["value"], decoded["text"]), (7, "7:Move middle point"))
        for no in I.READ_ONLY_PARAMETERS:
            with self.assertRaises(ValueError):
                I.encode_user_parameter(I.USER_PARAMETERS_BY_NO[no], 0, 0)

    def test_missing_item(self) -> None:
        named = I.decode_user_parameters({})
        self.assertEqual(len(named), 18)
        self.assertTrue(all(not p["found"] and p["text"] == "not found on this EC" for p in named))
        self.assertEqual([p["no"] for p in named], list(range(1, 19)))

    def test_gripper_baseline_through_the_simulator(self) -> None:
        ec, _ = open_ec("gripper")
        with ec:
            ec.identify()
            classes = {ci: I.read_class(ec, ci) for ci in I.PARAMETER_CLASSES}
        named = {p["no"]: p for p in I.decode_user_parameters(classes)}
        # IA-OS showed exactly these values for the gripper (VERIFIED)
        expected = {
            1: "9.50 mm", 2: "0.10 mm", 3: "1:Forward", 4: "0.50 mm", 5: "0:Disable", 6: "1:Enable",
            7: "0:Enable", 9: "0:Double", 10: "0:Disable", 12: "0:Continuous energization",
        }  # fmt: skip
        self.assertEqual({no: named[no]["text"] for no in expected}, expected)
        # P8 / P11 / P13-P18 are decoded too (UNVERIFIED locations; not on the gripper's list; read-only here)
        self.assertEqual(named[8]["text"], "0:Enable")
        self.assertEqual((named[14]["text"], named[15]["text"]), ("10.30 mm", "-0.30 mm"))
        self.assertTrue(all(named[no]["read_only"] for no in (8, 11, 13, 14, 15, 16, 17, 18)))
        self.assertEqual(named[4]["value"], 0.50)
        self.assertEqual((named[3]["address"], named[3]["raw_hex"]), ("0x0918", "00000007"))
        self.assertEqual(named[7]["address"], "0x083A")  # item 8.50 on the gripper ...
        self.assertEqual(named[6]["key"], "8.14")

    def test_slider_items_found_by_id(self) -> None:
        ec, _ = open_ec("slider")
        with ec:
            ec.identify()
            classes = {ci: I.read_class(ec, ci) for ci in (7, 8, 9)}
        named = {p["no"]: p for p in I.decode_user_parameters(classes)}
        self.assertEqual(named[7]["address"], "0x083E")  # ... and 0x083E on the slider (39 items in class 8)
        self.assertEqual(named[1]["text"], "150.00 mm")
        self.assertEqual(named[9]["text"], "4:3-position")
        self.assertEqual(named[4]["text"], "2.00 mm")


# =========================================================================== classes and blocks


class ReadClassTest(unittest.TestCase):
    def setUp(self) -> None:
        self.ec, self.sim = open_ec("gripper")
        self.ec.identify()

    def tearDown(self) -> None:
        self.ec.close()

    def test_items_by_id(self) -> None:
        c8 = I.read_class(self.ec, 8)
        self.assertEqual((c8.address, c8.words, len(c8.items)), (0x0800, 68, 34))
        self.assertEqual(c8.item(10).raw, 5300)  # default / max speed 53.00 mm/s at 0x0812
        self.assertEqual(c8.item(10).address, 0x0812)
        self.assertIsNone(c8.item(26))  # not in the gripper's class 8
        data = c8.to_dict()
        self.assertEqual(data["id_map"], "0x5200")
        first = data["items"][0]
        self.assertEqual((first["key"], first["hex"], first["unsigned"]), ("8.1", "20400000", 0x20400000))
        c7 = I.read_class(self.ec, 7)
        lower = c7.item(2)
        self.assertEqual((lower.signed, lower.to_dict()["scaled"]), (-30, -0.3))

    def test_multi_entry_and_16_bit_classes(self) -> None:
        zones = I.read_class(self.ec, 0x000C)  # 2 items x 2 entries
        self.assertEqual([i.key for i in zones.items], ["C.1", "C.2", "C.1[1]", "C.2[1]"])
        self.assertEqual(zones.item(1, entry=1).address, 0x0C04)
        control = I.read_class(self.ec, 0x000D)  # MS = 1
        self.assertEqual(control.item(1).raw, 0x1000)  # DRG1: SON
        self.assertEqual((control.item(1).size, control.item(1).hex), (1, "1000"))
        self.assertEqual(control.item(3).raw, 0x0100)  # DRGE: PMSL
        monitor = I.read_class(self.ec, 0x0101)  # mixed sizes, no ID map: word by word
        self.assertEqual(len(monitor.items), 34)
        self.assertTrue(all(i.item_id is None for i in monitor.items))

    def test_refuses_unknown_and_huge_classes(self) -> None:
        with self.assertRaises(LookupError):
            I.read_class(self.ec, 0x0112)  # the gripper has no 0x9E00 block
        before = len(self.sim.requests)
        with self.assertRaises(ValueError):
            I.read_class(self.ec, R.CATEGORY_POSITION_TABLE)  # 12288 registers
        self.assertEqual(len(self.sim.requests), before)  # nothing was read

    def test_read_block_bounds(self) -> None:
        self.assertEqual(len(I.read_block(self.ec, R.CATEGORY_COMMAND2)), 10)  # 0x9900-0x9909
        self.assertEqual(I.read_block(self.ec, R.CATEGORY_MAINTENANCE, 0, 4), [0, 9752, 0, 88])
        before = len(self.sim.requests)
        for offset, count in ((0, 11), (9, 2), (-1, 1), (0, 0)):
            with self.assertRaises(ValueError):
                I.read_block(self.ec, R.CATEGORY_COMMAND2, offset, count)
        self.assertEqual(len(self.sim.requests), before)
        self.assertEqual(I.read_block(self.ec, 0x0100), [0] * 32)  # in the directory, no recorded data: 0

    def test_read_block_uncertain_tail(self) -> None:
        """A block of uncertain size can be read up to its largest extent: live words sit past DQ x LQ."""
        ec, sim = open_ec("slider")
        with ec:
            _, records = I.read_directory(ec)
            by_class = {r.category_id: r for r in records}
            # 0x9100 (class 0x0109, DQ 15): 15 words certain, the slider's live 910F-9113 lie past them
            self.assertEqual(I.block_extent(by_class[0x0109]), (15, False, 30))
            self.assertEqual(I.readable_words(by_class[0x0109], records), 30)
            self.assertEqual(I.read_block(ec, 0x0109, 15, 5, records=records), [0x1A10, 0x292A, 0x2B10, 0x0000, 0x8008])
            self.assertEqual(len(I.read_block(ec, 0x0109, records=records)), 15)  # default: the certain extent
            self.assertEqual(I.read_block(ec, 0x0012, 8, 1, records=records), [0x4800])  # 0x7F08
            self.assertEqual(I.read_block(ec, 0x0112, 20, 2, records=records), [0x1402, 0x0544])  # 0x9E14-15
            # compact monitor (known extent): 9F18 OLLV, 9F1A-1B TLMC, 9F1C-1D ODOM
            self.assertEqual(I.read_block(ec, 0x0114, 24, 6, records=records), [0x0012, 0, 0, 0x0065, 0, 0x0002])
            before = len(sim.requests)
            for class_id, offset, count in ((0x0109, 29, 2), (0x0109, 30, 1), (0x0114, 39, 2)):
                with self.assertRaises(ValueError, msg=(hex(class_id), offset, count)):
                    I.read_block(ec, class_id, offset, count, records=records)
            self.assertEqual(len(sim.requests), before)  # refused before anything was sent
            summary = {r["class"]: r for r in I.directory_summary(None, records)["records"]}
            self.assertEqual((summary["0x0109"]["words"], summary["0x0109"]["words_readable"]), (15, 30))
            self.assertEqual(summary["0x0101"]["words_readable"], 34)  # exact: the extent itself

    def test_readable_words_never_reach_the_next_block(self) -> None:
        a = I.IndexCategory(0x0777, 0, 10, 1, 0xA000, 0, 0, 0)  # unknown mixed block: 10 certain, 20 at most
        b = I.IndexCategory(0x0778, 0, 4, 1, 0xA00C, 0, 0, 1)  # the next block starts 12 words later
        self.assertEqual(I.readable_words(a), 20)
        self.assertEqual(I.readable_words(a, (a, b)), 12)

    def test_directory_without_identify(self) -> None:
        ec, _ = open_ec("slider")
        with ec:
            self.assertEqual(ec.categories, ())
            header, records = I.read_directory(ec)
            self.assertEqual((header.record_count, len(records)), (27, 27))
            self.assertEqual(I.read_class(ec, 7).item(1).raw, 15030)  # reads the directory itself
            summary = I.directory_summary(header, records)
        self.assertEqual(summary["record_count"], 27)
        monitor = next(r for r in summary["records"] if r["class"] == "0x0101")
        self.assertEqual((monitor["address"], monitor["end"], monitor["exact"]), ("0x9000", "0x9021", True))
        compact = next(r for r in summary["records"] if r["class"] == "0x0114")
        self.assertEqual((compact["words"], compact["exact"], compact["end"]), (40, True, "0x9F27"))
        unknown = next(r for r in summary["records"] if r["class"] == "0x0112")
        self.assertEqual((unknown["words"], unknown["exact"], unknown["words_max"]), (18, False, 36))


# =========================================================================== alarm names


class AlarmNameTest(unittest.TestCase):
    def test_names(self) -> None:
        self.assertEqual(alarm_name(0x0EE), "ABS error (2)")
        self.assertEqual(alarm_name(0x0C1), "Servo error")
        self.assertEqual(alarm_name(0x0FFF), "PowerUP (no error)")  # the driver's curated name wins
        self.assertEqual(alarm_name(0x04E), "Total moving count is exceeded threshold.")  # the alarm-name table
        self.assertEqual(alarm_name(0), "No alarm (in normal condition)")
        self.assertEqual(alarm_name(0x021), "alarm 0x021")  # a command-rejection code, not an alarm
        self.assertEqual(alarm_name(0x1234), "alarm 0x1234")
        self.assertTrue(all(code == 0 or 0x040 <= code <= 0xFFF for code in CONTROLLER_ALARM_NAMES))
        self.assertEqual(AlarmRecord(0, 0x048, 0, 0xFFFF, None).name, "Driver overload warning")


# =========================================================================== unit_info on the simulator


class UnitInfoSliderTest(unittest.TestCase):
    def setUp(self) -> None:
        self.ec, self.sim = open_ec("slider")
        self.data = self.ec.unit_info()

    def tearDown(self) -> None:
        self.ec.close()

    def test_complete_and_serialisable(self) -> None:
        self.assertEqual(self.data["errors"], [])
        self.assertEqual(self.data["meta"]["schema"], "ec_driver.unit_info/1")
        self.assertEqual(self.data["meta"]["sections"], list(I.SECTIONS))
        self.assertEqual(json.loads(json.dumps(self.data))["identity"]["model"], "MWB-EC-DSRR4-WA-MF")

    def test_read_only(self) -> None:
        self.assertLessEqual({f[1] for f in self.sim.requests}, READ_ONLY_FUNCTIONS)
        self.assertEqual(self.sim.nonvolatile_writes, 0)

    def test_identity(self) -> None:
        ident = self.data["identity"]
        self.assertEqual(ident["serial_number"], "SIM-0000001")  # placeholder, never the real serial
        self.assertEqual((ident["unit_code"], ident["firmware"], ident["slave"]), (0x02, "0013", 1))
        self.assertEqual(ident["soft_limits_mm"], [-0.3, 150.3])
        self.assertEqual(ident["stroke_mm"], [0.0, 150.0])
        self.assertEqual((ident["ec_max_speed_mm_s"], ident["speed_cap_mm_s"]), (700.0, 700.0))  # cap = MAXV
        self.assertIsNone(ident["baudrate"])  # simulator: no serial port
        self.assertEqual(ident["position_mode"]["layout"], "3pos")
        self.assertEqual(ident["directory"]["record_count"], 27)
        self.assertEqual(ident["identification_extra"]["date_guess"], "2025-09-26")

    def test_parameters(self) -> None:
        params = self.data["parameters"]
        self.assertEqual(
            [c["class"] for c in params["classes"]], ["0x0003", "0x0004", "0x0007", "0x0008", "0x0009", "0x000C"]
        )
        c8 = next(c for c in params["classes"] if c["class"] == "0x0008")
        self.assertEqual(len(c8["items"]), 39)
        p14 = next(i for i in c8["items"] if i["key"] == "8.14")
        self.assertEqual(p14["params"], ["P5", "P6"])
        speed = next(i for i in c8["items"] if i["key"] == "8.10")
        self.assertEqual(speed["scaled"], 700.0)
        self.assertIn("default command speed", speed["note"])  # 8.10 VCMD: the maximum is MAXV 7.4 (VERIFIED on the rod)
        c7 = next(c for c in params["classes"] if c["class"] == "0x0007")
        maxv = next(i for i in c7["items"] if i["key"] == "7.4")
        self.assertEqual((maxv["scaled"], maxv["address"]), (700.0, "0x0706"))
        self.assertIn("MAXV", maxv["note"])
        unknown = next(i for i in c8["items"] if i["key"] == "8.105")  # unmapped items are still listed raw
        self.assertEqual(unknown["params"], [])

    def test_io_status_monitor(self) -> None:
        io_ = self.data["io"]
        self.assertEqual(io_["layout"], "3pos")
        self.assertEqual([b["name"] for b in io_["DOPM"]["bits"]], ["*ALM", "LS2", "LS1", "LS0"])
        self.assertEqual(io_["DOPM"]["on"], ["*ALM"])  # 73.95 mm: no marker, no alarm
        self.assertEqual(io_["lamps"], {"backward_end": False, "forward_end": False, "middle": False, "alarm": False,
                                        "warning": False})
        words = self.data["status_words"]
        self.assertEqual(words["DSS1"]["on"], ["PWR", "SV", "BKRL", "HEND", "PEND"])
        self.assertIn("PMSS", words["DSSE"]["on"])
        self.assertEqual(words["SSSE"]["value"], 0x1008)
        self.assertEqual(words["STAT"]["hex"], "0x0000801F")
        mon = self.data["monitor"]
        self.assertEqual((mon["position_mm"], mon["alarm_code"], mon["overload_level"]), (73.95, 0, 18))
        self.assertIn("uptime_ms", mon)
        control = self.data["control_words"]
        self.assertEqual(control["DRG1"]["on"], ["SON"])
        self.assertEqual(control["DRGE"]["on"], ["PMSL"])

    def test_alarms(self) -> None:
        alarms = self.data["alarms"]
        counts = (alarms["slots"], alarms["alarm_count"], alarms["power_up_markers"], alarms["empty_slots"])
        self.assertEqual(counts, (32, 5, 4, 23))
        self.assertEqual([a["name"] for a in alarms["history"]], ["ABS error (2)"] * 2 + ["Servo error"] * 3)
        first = alarms["history"][0]
        fields = (first["index"], first["hex"], first["detail"], first["time_raw"], first["newer_power_ups"])
        self.assertEqual(fields, (1, "0x0EE", 3, 0x000E84DB, 1))
        # UNVERIFIED: the time stamp is ms since power-on; 0x0EE is operation cancelled on an absolute (WA) unit
        self.assertEqual((first["time_ms"], first["time_after_power_on"]), (951515, "00:15:51.515"))
        self.assertEqual((first["level"], first["group"]), ("operation_cancelled", "D"))
        servo = alarms["history"][2]
        self.assertEqual((servo["hex"], servo["level"], servo["group"]), ("0x0C1", "operation_cancelled", "A"))
        cur = alarms["current"]
        self.assertEqual((cur["code"], cur["active"], cur["level"], cur["fault"], cur["warning"], cur["class3_code"]),
                         (0, False, None, False, False, 0))
        self.assertIn("newest", alarms["notes"]["order"])

    def test_maintenance_and_positions(self) -> None:
        maint = self.data["maintenance"]
        self.assertEqual((maint["total_moves"], maint["distance_m"]), (101, 2))
        temp = next(i for i in maint["items"] if i["symbol"] == "TEMP")
        self.assertEqual((temp["address"], temp["hi16"], temp["lo16"], temp["confirmed"]), ("0x840E", 39, 48, False))
        self.assertTrue(next(i for i in maint["items"] if i["id"] == 10)["unset"])
        pos = self.data["positions"]
        self.assertEqual(
            [(r["role"], r["position_mm"], r["speed_mm_s"]) for r in pos["rows"]],
            [("backward end", 0.0, 108.5), ("forward end", 150.0, 248.5), ("middle", 75.0, 248.5)],
        )
        self.assertTrue(all(r["checksum_ok"] for r in pos["rows"]))
        self.assertEqual(pos["rows"][1]["ctlf_on"], ["GSL0"])  # the unknown 0x0010 role bit of row 1
        simple = pos["simple_data_rows"]
        self.assertEqual([r["n"] for r in simple], [260, 261, 262, 263, 266])  # 3-position: the Middle % is 266
        self.assertEqual(simple[3]["role"], "forward end, previous (IA-OS %)")  # 263 is not the Middle (UNVERIFIED)
        self.assertEqual((simple[4]["role"], simple[4]["position_mm"]), ("middle, present (IA-OS %)", 75.0))
        self.assertEqual([(r["speed_pct"], r["accel_pct"], r["decel_pct"]) for r in simple],
                         [(16, 30, 30), (71, 60, 60), (71, 60, 60), (71, 60, 60), (71, 60, 60)])  # Bwd 30/16/30, Fwd 60/71/60
        self.assertTrue(all(r["checksum_ok"] for r in simple))
        dm = pos["direct_move"]
        self.assertEqual((dm["target_mm"], dm["band_mm"], dm["speed_mm_s"], dm["accel_g"]), (73.95, 0.1, 700.0, 0.3))
        self.assertEqual(pos["active_profile"]["dpow"], 255)

    def test_reads_stay_inside_the_directory(self) -> None:
        assert_reads_inside_directory(self, self.ec, self.sim)


class UnitInfoGripperTest(unittest.TestCase):
    def setUp(self) -> None:
        self.ec, self.sim = open_ec("gripper")

    def tearDown(self) -> None:
        self.ec.close()

    def test_gripper(self) -> None:
        data = self.ec.unit_info()
        self.assertEqual(data["errors"], [])
        self.assertLessEqual({f[1] for f in self.sim.requests}, READ_ONLY_FUNCTIONS)
        ident = data["identity"]
        self.assertEqual((ident["model"], ident["serial_number"]), ("EC-GRBP8M-20-1-1-PN-WA-WL2", "SIM-0000002"))
        self.assertEqual((ident["unit_code"], ident["firmware"]), (0x0A, "0002"))
        self.assertEqual(ident["directory"]["record_count"], 25)
        self.assertEqual(ident["position_mode"]["name"], "2-position (double solenoid)")
        named = {p["no"]: p["text"] for p in data["parameters"]["named"]}
        self.assertEqual((named[3], named[4], named[6], named[7]), ("1:Forward", "0.50 mm", "1:Enable", "0:Enable"))
        io_ = data["io"]
        self.assertEqual(io_["layout"], "2pos")
        self.assertEqual(io_["DOPM"]["on"], ["*ALM", "LS1"])  # 9.50 mm (live 0x0006)
        self.assertTrue(io_["lamps"]["forward_end"])
        self.assertEqual([b["name"] for b in io_["DIPM"]["bits"]], ["RES", "ST1", "ST0"])
        self.assertEqual(data["maintenance"]["total_moves"], 9752)
        self.assertEqual(data["maintenance"]["distance_m"], 88)
        alarms = data["alarms"]
        self.assertEqual((alarms["alarm_count"], alarms["power_up_markers"]), (6, 4))
        self.assertTrue(all(a["name"] == "Servo error" for a in alarms["history"]))
        rows = data["positions"]["rows"]
        push = (rows[1]["push"], rows[1]["push_end_mm"], rows[1]["push_direction"])
        self.assertEqual(push, (True, 9.5, "away from home"))
        self.assertEqual(rows[2]["role"], "unused in 2-position mode")
        self.assertEqual(data["positions"]["simple_data_rows"][3]["role"], "forward end, previous (IA-OS %)")
        self.assertEqual(len(data["positions"]["simple_data_rows"]), 4)  # 2-position: no Middle row 266
        assert_reads_inside_directory(self, self.ec, self.sim)

    def test_alarm_and_move_counter_follow_the_unit(self) -> None:
        self.ec.move_to(5.0, speed_mm_s=20.0)
        self.sim.trigger_alarm(0x0E0)
        data = self.ec.unit_info(sections=["alarms", "maintenance"])
        self.assertIsNone(data["identity"])
        self.assertEqual(data["maintenance"]["total_moves"], 9753)  # TLMC counts move starts
        cur = data["alarms"]["current"]
        self.assertEqual((cur["code"], cur["name"], cur["active"]), (0x0E0, "Over load", True))
        self.assertEqual((cur["level"], cur["class3_code"]), ("cold_start", 0x0E0))  # 0x0E0 Over load
        self.assertTrue(cur["fault"] and not cur["warning"])
        self.assertIn("software reset", cur["reset"])
        self.assertEqual(cur["group"], "A")
        newest = data["alarms"]["history"][0]
        self.assertEqual((newest["index"], newest["code"], newest["newer_power_ups"]), (0, 0x0E0, 0))


class UnitInfoEdgeTest(unittest.TestCase):
    def test_sections(self) -> None:
        ec, sim = open_ec()
        with ec:
            data = ec.unit_info(sections=["maintenance"])
            self.assertEqual(data["meta"]["sections"], ["maintenance"])
            self.assertIsNone(data["parameters"])
            self.assertEqual(data["maintenance"]["total_moves"], 101)
            data = ec.unit_info(sections=("monitor",))
            self.assertEqual(data["io"]["layout"], "3pos")  # P9 is read for the I/O layout
            self.assertIsNone(data["alarms"])
            with self.assertRaises(ValueError):
                ec.unit_info(sections=["everything"])

    def test_unreadable_block_is_reported_not_raised(self) -> None:
        ec, _ = open_ec(soft_limits=False)  # class 7 (0x0700) answers exception 02
        with ec:
            data = ec.unit_info()
        self.assertTrue(any("class 0x0007" in e for e in data["errors"]), data["errors"])
        named = {p["no"]: p for p in data["parameters"]["named"]}
        self.assertFalse(named[1]["found"])
        self.assertEqual(named[2]["text"], "0.10 mm")  # the other classes are still there
        self.assertIsNotNone(data["alarms"])

    def test_no_link_raises(self) -> None:
        ec, sim = open_ec()
        sim.inject_fault("timeout", 1000)
        with ec, self.assertRaises(ModbusTimeout):
            ec.unit_info()

    def test_decode_monitor_from_a_status_poll(self) -> None:
        """The GUI's live view: Status.raw (22 words) decodes like unit_info's monitor sections."""
        for profile, layout, dopm_b2 in (("slider", "3pos", "LS2"), ("gripper", "2pos", "*ALM")):
            ec, sim = open_ec(profile)
            with self.subTest(profile=profile), ec:
                data = ec.unit_info(sections=["monitor"])
                mode = I.position_mode(4 if profile == "slider" else 0)
                sim.requests.clear()
                live = I.decode_monitor(ec.status().raw, mode)
                self.assertEqual({f[1] for f in sim.requests}, {0x03})  # the status read only
                self.assertEqual(set(live), {"monitor", "status_words", "io"})
                for name in ("DSS1", "DSS2", "DSSE", "ZONS", "STAT", "SIPM"):
                    self.assertEqual(live["status_words"][name]["bits"], data["status_words"][name]["bits"], name)
                self.assertIn("SSSE", live["status_words"])  # in the 22-word poll (0x9000-0x9015)
                self.assertIn("POSS", live["status_words"])
                self.assertEqual(live["io"]["layout"], layout)
                self.assertEqual(next(b["name"] for b in live["io"]["DOPM"]["bits"] if b["bit"] == 2), dopm_b2)
                self.assertEqual(live["io"]["lamps"], data["io"]["lamps"])
                self.assertEqual(live["monitor"]["position_mm"], data["monitor"]["position_mm"])
                self.assertIn("uptime", live["monitor"])
                self.assertNotIn("overload_level", live["monitor"])
        empty = I.decode_monitor({})
        self.assertEqual((empty["status_words"], empty["io"]["lamps"], empty["io"]["layout"]), ({}, {}, "2pos"))


class EditionWordingTest(unittest.TestCase):
    """What the package writes: the 1.0 text, "this unit" for text that applies to any unit, IA-OS
    where the tool is named, and a whole float still a float (1.0, not 1)."""

    def test_identity_notes_and_limits(self) -> None:
        ec, _ = open_ec("unknown")
        with ec:
            data = ec.unit_info(sections=["identity"])
        self.assertIn("AKID kind 42: commanded as a linear axis in mm",
                      data["identity"]["notes"])  # fmt: skip
        self.assertFalse(any("driver's table" in n for n in data["identity"]["notes"]))
        limits = data["identity"]["capabilities"]["axis_limits"]
        self.assertNotIn("notes", limits)
        for key in ("max_speed_mm_s", "max_accel_g", "max_decel_g"):
            self.assertIsInstance(limits[key], float, key)

    def test_parameter_notes_and_evidence(self) -> None:
        ec, _ = open_ec("rod")
        with ec:
            data = ec.unit_info(sections=["parameters"])
        notes = {i["key"]: i["note"] for c in data["parameters"]["classes"] for i in c["items"] if i.get("note")}
        self.assertEqual(notes["8.10"], "VCMD: default command speed (0x0812; not the maximum, which is MAXV 7.4)")
        self.assertEqual(notes["4.5"], "MCUR: rated current (IA-OS shows CNOW / MCUR x 100 %)")
        evidence = {p["no"]: p["evidence"] for p in data["parameters"]["named"]}
        self.assertTrue(evidence[1].startswith("VERIFIED on the EC-GRBP8M gripper (fw 0002): changed one at a time in "
                                               "IA-OS"), evidence[1])  # fmt: skip
        self.assertIn("(IA-OS hid it on all units tested)", evidence[13])

    def test_model_note_and_directory_wording(self) -> None:
        ec, _ = open_ec("gripper")
        with ec:
            data = ec.unit_info(sections=["parameters"])
            with self.assertRaisesRegex(LookupError, "class 0x0112 is not in this unit's directory"):
                I.read_class(ec, 0x0112)
        self.assertTrue(data["parameters"]["model"]["note"].startswith("the parameters IA-OS lists for unit 0x0A"))


class ParameterRowsTest(unittest.TestCase):
    """The rows of parameters() / read_parameter() and what a parameter write says: the keys
    a Python reader finds, ints where 1.0 held ints and floats (1.0, not 1) where it held floats."""

    RAW_ITEM_FIELDS = {"unsigned", "signed", "words", "named"}

    def test_an_ec_row_has_no_raw_item_fields_and_keeps_its_number_types(self) -> None:
        ec, _ = open_ec("gripper")
        with ec:
            rows = {p["number"]: p for p in ec.parameters()}
            read = ec.read_parameter(5)
        for number, row in rows.items():
            self.assertFalse(self.RAW_ITEM_FIELDS & set(row), (number, sorted(row)))
            self.assertNotIn("symbol", row)  # only the CON parameters have one
            self.assertNotEqual(row.get("note"), "", number)  # a note only when there is one
        self.assertFalse(self.RAW_ITEM_FIELDS & set(read))
        for key in ("value", "min", "max"):  # a code, a count: int
            self.assertIsInstance(rows[5][key], int, key)
            self.assertIsInstance(rows[9][key], int, key)
            self.assertIsInstance(read[key], int, key)
        for number in (1, 2):  # a measure in mm: float, even when whole
            for key in ("value", "min", "max"):
                self.assertIsInstance(rows[number][key], float, f"P{number} {key}")

    def test_a_con_row_has_its_symbol_and_the_types_of_the_table(self) -> None:
        ec, _ = open_ec("pcon")
        with ec:
            rows = {p["number"]: p for p in ec.parameters()}
            numbered = {p["number"]: p for p in ec.unit_info(sections=["parameters"])["parameters"]["numbered"]}
        self.assertEqual(rows[8]["symbol"], "VCMD")
        for number, key, kind in ((8, "min", int), (8, "max", int), (8, "value", float),  # scaled: a float value
                                  (16, "value", int), (16, "min", int),  # a baud rate, a count: ints
                                  (22, "min", float), (22, "max", float),  # the table writes 0.0 and 9999.99
                                  (55, "min", float), (55, "max", float)):  # fmt: skip
            self.assertIsInstance(rows[number][key], kind, f"No.{number} {key}")
            self.assertIsInstance(numbered[number][key], kind, f"numbered No.{number} {key}")
        self.assertFalse(self.RAW_ITEM_FIELDS & set(numbered[8]))
        for number, key, kind in ((48, "factory_default", float), (65, "factory_default", int),
                                  (55, "family_min", float), (55, "family_max", float),
                                  (8, "family_min", int), (16, "family_max", int)):  # fmt: skip
            self.assertIsInstance(numbered[number][key], kind, f"numbered No.{number} {key}")

    def test_a_parameter_that_is_not_offered_is_named_like_the_key(self) -> None:
        ec, _ = open_ec("gripper")
        with ec:
            with self.assertRaisesRegex(UnsafeCommandError, r"^parameter 15 is not offered on this unit$"):
                ec.read_parameter(15)
            with self.assertRaisesRegex(UnsafeCommandError, r"^parameter 'P15' is not offered on this unit$"):
                ec.read_parameter("P15")

    def test_a_write_says_what_the_python_edition_says(self) -> None:
        ec, _ = open_ec("gripper")
        with ec:
            ram = ec.write_parameter(5, 0, commit=False)
            committed = ec.write_parameter(6, 1 - ec.read_parameter(6)["value"], commit=True)
            unlisted = ec.write_parameter(12, 1, commit=False, allow_unlisted=True)
        self.assertTrue(ram.note.startswith("RAM ONLY - NOT SAVED (commit=False): a parameter written over Modbus (FC10) "
                                            "changes the EC's RAM only and reverts"), ram.note)
        self.assertTrue(committed.note.startswith("Committed to non-volatile memory with coil 0x0428 (DRGE b7 PARW), the "
                                                  "coil IA-OS's Transfer writes (ON-WIRE: USB capture of IA-OS)."),
                        committed.note)  # fmt: skip
        self.assertEqual(unlisted.warnings[0], "P12 is not in ME3816-6B (IA-OS lists it): its meaning is UNVERIFIED")

    def test_a_ranges_stored_rows_and_speed_override_are_ints(self) -> None:
        ec, _ = open_ec("gripper")
        with ec:
            limits = ec.limits()
        for name in ("stored_rows", "speed_override"):
            for key in ("min", "max"):
                self.assertIsInstance(limits[name][key], int, f"{name} {key}")
        self.assertIsInstance(limits["position"]["max"], float)
        self.assertIsInstance(limits["push"]["min"], float)


class SimulatorInfoBlocksTest(unittest.TestCase):
    def test_directory_bounds_and_dynamic_blocks(self) -> None:
        for profile, dopm in (("slider", 0x0008), ("gripper", 0x0006)):
            sim = SimulatedEC(profile=profile)
            bus = ModbusRTU(sim, min_gap_s=0.0)
            with self.subTest(profile=profile):
                for address in (0x9022, 0x7000, 0x0C08, 0x0136):  # outside every directory block / ID map
                    with self.assertRaises(Exception) as ctx:
                        bus.read_registers(1, address, 1)
                    self.assertEqual(getattr(ctx.exception, "code", None), 0x02)
                image = bus.read_registers(1, 0x0200, 10)
                status = bus.read_registers(1, R.MONITOR_BASE, 20)
                self.assertEqual(image[0], status[5])  # DSS1
                self.assertEqual(image[2], status[7])  # DSSE
                self.assertEqual(image[4], status[19])  # ZONS
                self.assertEqual(image[6], dopm)  # DOPM
                sim.trigger_alarm(0x0EE)
                self.assertEqual(bus.read_registers(1, 0x0312, 2), [0, 0x0EE])  # class 3 item 10 at 0x0312
        gripper = ModbusRTU(SimulatedEC(profile="gripper"), min_gap_s=0.0)
        self.assertEqual(len(gripper.read_registers(1, 0x0800, 68)), 68)  # the gripper's 34 class-8 items
        with self.assertRaises(Exception):
            gripper.read_registers(1, 0x0844, 1)  # the slider's longer class 8 is not served here


class FacadePatchTest(unittest.TestCase):
    def test_a_patch_of_the_public_name_reaches_the_running_code(self) -> None:
        # backup_axes calls backup by its module name: a patch of iai_actuator_driver.info.backup reaches it (the
        # shared backup_axes runs with this module's names)
        ec, _ = open_ec("rod")
        failing = mock.patch("iai_actuator_driver.info.backup", side_effect=OSError("patched"))
        with ec, tempfile.TemporaryDirectory() as tmp, failing as patched:
            paths = I.backup_axes({"rod": ec}, tmp)
        patched.assert_called_once()
        self.assertEqual(paths, {"rod": "error: OSError: patched"})


# =========================================================================== CLI and iai-check


class InfoCliTest(unittest.TestCase):
    def test_text(self) -> None:
        factory = SimFactory()
        code, out, _ = run("--sim", "info", factory=factory)
        self.assertEqual(code, 0)
        for text in ("[identity]", "model         MWB-EC-DSRR4-WA-MF", "serial        SIM-0000001",
                     "position mode 3-position", "[user parameters]", "1:Forward", "4:3-position",
                     "[I/O] 3-position", "DSS1  0x3098", "[alarms]", "ABS error (2)", "total moves   101",
                     "row 263 forward end, previous (IA-OS %)", "row 266 middle, present (IA-OS %)",
                     "actuator      slider (small) (AKID 9)", "axis number   0 (AXNO 0x8004)",
                     "collision     not reported (MVPT b13 CDAS = 0)", "boot baud     38400 (BRSL item 8.3)",
                     "0x9900 last"):  # fmt: skip
            self.assertIn(text, out)
        self.assertNotIn("[not read]", out)
        self.assertLessEqual({f[1] for f in factory.instances[0].requests}, READ_ONLY_FUNCTIONS)

    def test_json_either_position_and_sections(self) -> None:
        code, out, _ = run("--sim", "--sim-profile", "gripper", "info", "--json")
        self.assertEqual(code, 0)
        data = json.loads(out)
        self.assertEqual({p["no"]: p["text"] for p in data["parameters"]["named"]}[6], "1:Enable")
        code, out, _ = run("--sim", "--json", "info", "--section", "maintenance", "--section", "alarms")
        self.assertEqual(code, 0)
        data = json.loads(out)
        self.assertEqual(data["meta"]["sections"], ["maintenance", "alarms"])
        self.assertIsNone(data["positions"])
        code, out, _ = run("--sim", "info", "--section", "monitor")
        self.assertIn("[monitor]", out)
        self.assertNotIn("[identity]", out)

    def test_check_report_carries_unit_info(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "report.json"
            code, out, _ = run("--sim", "check", "--report", str(path))
            self.assertEqual(code, 0, out)
            self.assertIn("PASS  unit_info", out)
            report = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(report["unit_info"]["meta"]["schema"], "ec_driver.unit_info/1")
        self.assertEqual(report["unit_info"]["maintenance"]["total_moves"], 101)


if __name__ == "__main__":
    unittest.main()
