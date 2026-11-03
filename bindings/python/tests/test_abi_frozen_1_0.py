"""The frozen ABI 1.0 files of core/abi/1.0/ are never edited: their sha256 is pinned here.

``core/abi/1.0/`` holds the stable header of C ABI 1.0 (as released in 1.0.0) and the C smoke program built with it.
``python build.py smoke`` and both SDK smoke steps of the local release build that program against this header only and
run it on the current library: a C program built for 1.0 must still run on every later 1.x library. The proof holds only
while the two files stay what 1.0 shipped, so a change to either one is a change of the proof, never of this test: a
new major gets a folder of its own (core/abi/2.0/) and this one stays. The tests need a source checkout.
"""

from __future__ import annotations

import hashlib
import re
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
FROZEN = REPO / "core" / "abi" / "1.0"

# sha256 of the bytes of each file (the repository keeps LF line endings: .gitattributes)
PINNED = {
    "iai_actuator_driver.h": "d64b313701f308cafab70e19d18fe6825240d0a476030dd40b26f615ca9efeff",
    "c_smoke.c": "c6f27e964e77ccd3775d219b2d55caab7a42bb0a3ffcd853a6a4053642aba3e2",
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


@unittest.skipUnless(FROZEN.is_dir(), "core/abi/1.0 is in a source checkout only")
class FrozenAbi10Test(unittest.TestCase):
    def test_the_files_are_the_pinned_ones(self) -> None:
        for name, digest in PINNED.items():
            with self.subTest(name):
                self.assertEqual(sha256(FROZEN / name), digest, f"core/abi/1.0/{name} was edited: it is frozen")

    def test_nothing_else_is_in_the_folder(self) -> None:
        """A third file would be built or shadowed by accident: the folder is the header and the program only (and its
        README)."""
        self.assertEqual(sorted(p.name for p in FROZEN.iterdir()), sorted([*PINNED, "README.md"]))

    def test_the_header_is_abi_1_0(self) -> None:
        text = (FROZEN / "iai_actuator_driver.h").read_text(encoding="ascii")
        self.assertRegex(text, r"(?m)^#define IAD_ABI_VERSION_MAJOR 1$")
        self.assertRegex(text, r"(?m)^#define IAD_ABI_VERSION_MINOR 0$")

    def test_the_program_includes_the_header_beside_it(self) -> None:
        """A quoted include finds the file beside the source first: the program must name the header that way, so that
        the SDK's current header (on the include path of every consumer) cannot stand in for the frozen one."""
        text = (FROZEN / "c_smoke.c").read_text(encoding="ascii")
        self.assertNotIn("<iai_actuator_driver.h>", text)
        self.assertEqual(re.findall(r'(?m)^#include "([^"]+)"$', text), ["iai_actuator_driver.h"])


if __name__ == "__main__":
    unittest.main()
