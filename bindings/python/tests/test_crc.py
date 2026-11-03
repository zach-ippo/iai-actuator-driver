"""CRC-16/MODBUS and frame building against frames printed in IAI manual ME0162."""

from __future__ import annotations

import struct
import unittest

from ec_testutil import hexbytes

from iai_actuator_driver.modbus import build_frame, check_crc, crc16

# Complete frames copied from ME0162 (CRCs independently re-checked); the last four are not in the manual: they
# were sent to an EC on hardware and answered (VERIFIED)
MANUAL_FRAMES = [
    "01 03 90 00 00 0A E8 CD",  # read 9000h x10
    "01 03 90 00 00 02 E9 0B",  # read PNOW
    "01 03 04 00 00 0B FE 7C 83",  # PNOW response = 30.70 mm
    "01 03 02 30 98 AD EE",  # DSS1 response 3098h
    "01 03 14 00 00 00 00 00 00 00 00 6E 00 60 18 80 00 23 C7 00 00 00 19 18 A6",  # burst read response
    "01 05 04 03 FF 00 7D 0A",  # SON on
    "01 05 04 03 00 00 3C FA",  # SON off
    "01 05 04 07 FF 00 3C CB",  # ALRS on
    "01 05 04 07 00 00 7D 3B",  # ALRS off
    "01 05 04 0A FF 00 AD 08",  # STP on
    "01 05 04 0B 00 00 BD 38",  # HOME off
    "01 05 04 0B FF 00 FC C8",  # HOME on
    "01 05 04 27 FF 00 3D 01",  # PMSL on
    "01 05 04 2C FF 00 4C C3",  # STOP
    "01 06 0D 00 10 00 86 A6",  # DRG1 = SON
    "01 06 98 00 00 01 67 6A",  # POSR immediate move to 1
    "01 10 99 00 00 02 04 00 00 13 88 38 AF",  # move to 50.00 mm
    "01 10 99 00 00 07 0E 00 00 13 88 00 00 00 0A 00 00 27 10 00 1E 50 CF",  # 50 mm, 0.1, 100 mm/s, 0.3 G
    "01 10 99 00 00 07 AF 57",  # its response
    "01 10 10 C0 00 0F 84 F1",  # position-table write response
    "01 11 C0 2C",  # report slave id (VERIFIED)
    "01 03 00 00 00 08 44 0C",  # index header read (VERIFIED)
    "01 45 00 09 00 01 00 00 18 C2",  # 0x45 servo on (VERIFIED)
    "01 45 00 0A 00 00 00 00 0D 02",  # 0x45 home, op1 0 (VERIFIED: rejected 0x11 by the EC; op1 must be ORG)
]


class CrcTest(unittest.TestCase):
    def test_known_vector(self) -> None:
        self.assertEqual(struct.pack("<H", crc16(hexbytes("01 03 90 00 00 0A"))), hexbytes("E8 CD"))

    def test_standard_check_value(self) -> None:
        # CRC-16/MODBUS catalogue check value for "123456789"
        self.assertEqual(crc16(b"123456789"), 0x4B37)

    def test_empty(self) -> None:
        self.assertEqual(crc16(b""), 0xFFFF)

    def test_manual_frames(self) -> None:
        for text in MANUAL_FRAMES:
            with self.subTest(frame=text):
                frame = hexbytes(text)
                self.assertTrue(check_crc(frame))
                self.assertEqual(build_frame(frame[0], frame[1:-2]), frame)

    def test_corruption_detected(self) -> None:
        frame = bytearray(hexbytes(MANUAL_FRAMES[0]))
        frame[3] ^= 0x01
        self.assertFalse(check_crc(bytes(frame)))
        self.assertFalse(check_crc(b"\x01\x03"))


if __name__ == "__main__":
    unittest.main()
