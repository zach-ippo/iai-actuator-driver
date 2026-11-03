"""The update gate of core/abi/manifest.py: within an ABI major from 1 on, only additions pass.

``manifest.py update`` compares the committed manifest with the headers through ``breaking_changes``. It reads the
stable header's declarations by name (constants, types, callbacks, functions) and each struct as an ordered field list:
a sized input (it starts with struct_size and reserved_header) keeps its fields in order and may get fields appended at
its end; an output keeps its size, so only a reserved slot may be given fields of the same total size. These tests
change the committed manifest text (abi 1.x) in memory and need no library (the tool is in a source checkout only).

BaselineGateTest closes what that update gate leaves open (``--allow-break``, and a stable header that grows while the
minor version stays 0): the committed manifest.txt against the frozen core/abi/manifest-1.0.txt.
"""

from __future__ import annotations

import hashlib
import importlib.util
import sys
import unittest
from collections.abc import Callable
from pathlib import Path
from types import ModuleType

ROOT = Path(__file__).resolve().parent.parent
TOOL = ROOT.parents[1] / "core" / "abi" / "manifest.py"


def _tool() -> ModuleType:
    spec = importlib.util.spec_from_file_location("iad_abi_manifest_gate", TOOL)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules.setdefault("iad_abi_manifest_gate", module)
    spec.loader.exec_module(module)
    return module


@unittest.skipUnless(TOOL.is_file(), "the ABI manifest is in a source checkout only")
class ManifestGateTest(unittest.TestCase):
    tool: ModuleType
    old: str

    @classmethod
    def setUpClass(cls) -> None:
        cls.tool = _tool()
        cls.old = cls.tool.MANIFEST.read_text(encoding="utf-8")

    def fields_of(self, struct: str) -> list[str]:
        lines = self.old.splitlines()
        start = lines.index(f"struct {struct}") + 1
        end = start
        while end < len(lines) and lines[end].startswith("    "):
            end += 1
        return lines[start:end]

    def changed(self, struct: str, edit: Callable[[list[str]], list[str]]) -> str:
        """The manifest with `edit(fields)` applied to the field lines of one struct of the stable header."""
        lines = self.old.splitlines()
        start = lines.index(f"struct {struct}") + 1
        end = start + len(self.fields_of(struct))
        return "\n".join(lines[:start] + edit(lines[start:end]) + lines[end:]) + "\n"

    def breaks(self, new: str) -> list[str]:
        return list(self.tool.breaking_changes(self.old, new))

    def test_the_same_manifest_and_plain_additions_pass(self) -> None:
        self.assertEqual(self.breaks(self.old), [])
        # a new constant, type and function inside the stable section
        text = self.old.replace("const IAD_OK 0\n", "const IAD_OK 0\nconst IAD_NEW 7\n", 1)
        text = text.replace("type iad_bool int32_t\n", "type iad_bool int32_t\ntype iad_new int32_t\n", 1)
        first_fn = next(line for line in self.old.splitlines() if line.startswith("function "))
        text = text.replace(first_fn + "\n", first_fn + "\nfunction iad_axis_added void(void)\n", 1)
        self.assertNotEqual(text, self.old)
        self.assertEqual(self.breaks(text), [])
        # a whole new struct
        text = self.old.replace("function ", "struct iad_new_struct\n    uint32_t struct_size\n    uint32_t reserved_header\nfunction ", 1)
        self.assertEqual(self.breaks(text), [])

    def test_a_changed_or_removed_declaration_is_refused_by_its_name(self) -> None:
        first_fn = next(line for line in self.old.splitlines() if line.startswith("function "))
        self.assertTrue(self.breaks(self.old.replace(first_fn + "\n", "", 1)))
        self.assertTrue(self.breaks(self.old.replace(first_fn, first_fn.replace("(", "(double, ", 1), 1)))
        self.assertTrue(self.breaks(self.old.replace("const IAD_OK 0", "const IAD_OK 1")))
        self.assertTrue(self.breaks(self.old.replace("type iad_bool int32_t\n", "", 1)))
        self.assertTrue(self.breaks(self.old.replace("callback iad_log_sink void(", "callback iad_log_sink int32_t(", 1)))

    def test_the_committed_manifest_is_of_major_1(self) -> None:
        self.assertEqual(self.tool._parse_stable(self.old).major, 1)  # the gate applies from major 1 on

    def test_a_minor_release_may_raise_the_minor_version_constant(self) -> None:
        line = next(line for line in self.old.splitlines() if line.startswith("const IAD_ABI_VERSION_MINOR "))
        raised = self.old.replace(line, f"const IAD_ABI_VERSION_MINOR {int(line.split()[-1]) + 1}")
        self.assertNotEqual(raised, self.old)
        self.assertEqual(self.breaks(raised), [])
        self.assertTrue(self.tool.breaking_changes(raised, self.old))  # lowered again: refused

    def test_a_field_inserted_into_a_struct_is_refused(self) -> None:
        # a sized input and an output: a field as the second one (the sized input's second field is reserved_header)
        for struct in ("iad_move_options", "iad_status"):
            with self.subTest(struct=struct):
                broken = self.breaks(self.changed(struct, lambda f: [f[0], "    double inserted", *f[1:]]))
                self.assertTrue(broken)
                self.assertTrue(all(struct in line for line in broken), broken)
        # in the middle of an output, away from the header
        broken = self.breaks(self.changed("iad_status", lambda f: [*f[:3], "    double inserted", *f[3:]]))
        self.assertTrue(broken and "iad_status" in broken[0], broken)

    def test_reordered_fields_are_refused(self) -> None:
        def swap(fields: list[str]) -> list[str]:
            i = next(n for n, f in enumerate(fields) if f.endswith(" position_mm"))
            j = next(n for n, f in enumerate(fields) if f.endswith(" velocity_mm_s"))
            fields[i], fields[j] = fields[j], fields[i]
            return fields

        broken = self.breaks(self.changed("iad_status", swap))
        self.assertTrue(broken and "iad_status" in broken[0], broken)
        # the same for a sized input
        self.assertTrue(self.breaks(self.changed("iad_move_options", lambda f: [f[0], f[1], f[3], f[2], *f[4:]])))

    def test_a_removed_field_is_refused_even_when_another_struct_has_the_same_line(self) -> None:
        # "iad_bool wait" is a field of the servo, home, stop and change options alike
        def drop(fields: list[str]) -> list[str]:
            self.assertIn("    iad_bool wait", fields)
            return [f for f in fields if f != "    iad_bool wait"]

        for struct in ("iad_servo_options", "iad_home_options", "iad_stop_options", "iad_change_options"):
            with self.subTest(struct=struct):
                broken = self.breaks(self.changed(struct, drop))
                self.assertTrue(broken and struct in broken[0], broken)
        # a removed (here: renamed) struct
        self.assertTrue(self.breaks(self.old.replace("struct iad_status\n", "struct iad_status_renamed\n", 1)))

    def test_a_sized_input_may_get_fields_appended_at_its_end_only(self) -> None:
        text = self.changed("iad_move_options", lambda f: [*f, "    double appended", "    uint64_t more[2]"])
        self.assertEqual(self.breaks(text), [])
        # a retyped field and a field placed before the reserved slots are not appends
        self.assertTrue(self.breaks(self.changed("iad_move_options", lambda f: [f[0], f[1], "    int32_t speed", *f[3:]])))
        self.assertTrue(self.breaks(self.changed("iad_move_options", lambda f: [*f[:-1], "    double appended", f[-1]])))

    def test_an_output_keeps_its_size_and_only_a_reserved_slot_may_be_given_fields(self) -> None:
        # appended to an output: refused
        broken = self.breaks(self.changed("iad_status", lambda f: [*f, "    double appended"]))
        self.assertTrue(broken and "iad_status" in broken[0], broken)
        fields = self.fields_of("iad_status")
        slot = next(n for n, f in enumerate(fields) if "uint64_t reserved[" in f)
        count = int(fields[slot].split("[")[1].rstrip("]"))
        self.assertGreaterEqual(count, 2)

        # a reserved slot given a meaning, equal total size (reserved[N] -> one double and reserved[N-1]): accepted
        def give_meaning(f: list[str]) -> list[str]:
            return [*f[:slot], "    double meaning", f"    uint64_t reserved[{count - 1}]", *f[slot + 1 :]]

        self.assertEqual(self.breaks(self.changed("iad_status", give_meaning)), [])

        def wrong_size(f: list[str]) -> list[str]:
            return [*f[:slot], "    double meaning", f"    uint64_t reserved[{count}]", *f[slot + 1 :]]

        self.assertTrue(self.breaks(self.changed("iad_status", wrong_size)))

        def smaller(f: list[str]) -> list[str]:
            return [*f[:slot], f"    uint64_t reserved[{count - 1}]", *f[slot + 1 :]]

        self.assertTrue(self.breaks(self.changed("iad_status", smaller)))

    def test_the_gate_applies_only_within_a_major_from_1_on(self) -> None:
        text = self.changed("iad_status", lambda f: [f[0], "    double inserted", *f[1:]])
        self.assertTrue(self.breaks(text))
        old = self.old.replace("abi 1.", "abi 0.")
        self.assertEqual(self.tool.breaking_changes(old, text.replace("abi 1.", "abi 0.")), [])
        self.assertEqual(self.tool.breaking_changes(self.old, text.replace("abi 1.", "abi 2.")), [])

    def test_every_struct_of_the_committed_manifest_can_be_sized(self) -> None:
        # the gate sizes the fields to match a reserved slot (no implicit padding: a struct is the sum of its fields)
        stable = self.tool._parse_stable(self.old)
        callbacks = {k[1] for k in stable.simple if k[0] == "callback"}
        self.assertGreater(len(stable.structs), 40)
        for name, fields in stable.structs.items():
            with self.subTest(struct=name):
                self.assertEqual(sum(self.tool._field_size(stable, f, callbacks) for f in fields) % 8, 0)
        self.assertEqual(sum(self.tool._field_size(stable, f, callbacks) for f in stable.structs["iad_move_options"]), 80)


@unittest.skipUnless(TOOL.is_file(), "the ABI manifest is in a source checkout only")
class BaselineGateTest(unittest.TestCase):
    """The committed manifest.txt against the frozen 1.0 baseline: within major 1 the stable header only grows, and
    only with a new minor version. The red cases are texts derived from the baseline (it never changes, so they stay
    valid when manifest.txt moves on to 1.1)."""

    tool: ModuleType
    baseline: str

    @classmethod
    def setUpClass(cls) -> None:
        cls.tool = _tool()
        cls.baseline = cls.tool.BASELINE.read_text(encoding="utf-8")

    def minor(self, text: str) -> int:
        return int(self.tool._parse_stable(text).consts["IAD_ABI_VERSION_MINOR"])

    def stable_section(self, text: str) -> list[str]:
        """The lines of the stable header's section of a manifest text, its heading included."""
        lines = text.splitlines()
        start = next(i for i, line in enumerate(lines) if line.startswith(f"[{self.tool.STABLE}]"))
        end = next((i for i in range(start + 1, len(lines)) if lines[i].startswith("[")), len(lines))
        return lines[start:end]

    def gate(self, current: str) -> list[str]:
        """What stops `current` passing the baseline (empty: it passes)."""
        problems = list(self.tool.breaking_changes(self.baseline, current))
        if self.stable_section(current) != self.stable_section(self.baseline) and self.minor(current) == 0:
            problems.append("the stable section differs from manifest-1.0.txt while IAD_ABI_VERSION_MINOR is 0 "
                            "(an addition to the stable header needs a new minor version)")
        return problems

    def first_function(self) -> str:
        return next(line for line in self.baseline.splitlines() if line.startswith("function "))

    def with_function_added(self, text: str) -> str:
        first = self.first_function()
        return text.replace(first + "\n", first + "\nfunction iad_axis_added void(void)\n", 1)

    def with_minor(self, text: str, minor: int) -> str:
        text = text.replace("abi 1.0\n", f"abi 1.{minor}\n", 1)
        return text.replace("const IAD_ABI_VERSION_MINOR 0\n", f"const IAD_ABI_VERSION_MINOR {minor}\n", 1)

    def test_the_baseline_is_the_1_0_manifest(self) -> None:
        stable = self.tool._parse_stable(self.baseline)
        self.assertEqual((stable.major, stable.minor, self.minor(self.baseline)), (1, 0, 0))
        self.assertGreater(len([k for k in stable.simple if k[0] == "function"]), 300)  # the gate is not vacuous
        self.assertGreater(len(stable.structs), 40)
        self.assertEqual(hashlib.sha256(self.baseline.replace("\r\n", "\n").encode("utf-8")).hexdigest(),
                         "c984bc7ee00e4f4a8853844df0d43212d9c0b7f44084aef6e9e45f95c21bf4ac",
                         "core/abi/manifest-1.0.txt is frozen: never edit or regenerate it")

    def test_the_committed_manifest_keeps_the_baseline(self) -> None:
        current = self.tool.MANIFEST.read_text(encoding="utf-8")
        self.assertEqual(self.tool._parse_stable(current).major, 1,
                         "the ABI major changed: freeze its manifest as a new baseline and point this gate at it")
        self.assertEqual(self.tool.breaking_changes(self.baseline, current), [])  # whatever --allow-break was given
        self.assertTrue(self.stable_section(current) == self.stable_section(self.baseline) or self.minor(current) > 0,
                        "the stable section grew while IAD_ABI_VERSION_MINOR is 0: raise the minor version")
        self.assertEqual(self.gate(current), [])

    def test_the_baseline_itself_and_a_new_minor_with_additions_pass(self) -> None:
        self.assertEqual(self.gate(self.baseline), [])
        raised = self.with_minor(self.with_function_added(self.baseline), 1)
        self.assertNotEqual(raised, self.baseline)
        self.assertEqual(self.gate(raised), [])
        self.assertEqual(self.gate(self.with_minor(self.baseline, 1)), [])  # a minor that adds nothing

    def test_a_removed_1_0_line_is_red(self) -> None:
        removed = self.baseline.replace(self.first_function() + "\n", "", 1)
        self.assertNotEqual(removed, self.baseline)
        broken = self.tool.breaking_changes(self.baseline, removed)
        self.assertEqual(broken, [self.first_function()])
        self.assertEqual(self.gate(removed)[:1], broken)  # (at minor 0 the section rule fires as well)
        # a raised minor version does not excuse it (nor does --allow-break: the gate reads the committed text)
        self.assertEqual(self.gate(self.with_minor(removed, 1)), broken)
        changed = self.baseline.replace("const IAD_OK 0\n", "const IAD_OK 1\n", 1)
        self.assertTrue(self.gate(self.with_minor(changed, 1)))

    def test_an_added_stable_line_at_minor_0_is_red(self) -> None:
        added = self.with_function_added(self.baseline)
        self.assertNotEqual(added, self.baseline)
        self.assertEqual(self.tool.breaking_changes(self.baseline, added), [])  # an addition: not a break
        problems = self.gate(added)
        self.assertEqual(len(problems), 1, problems)
        self.assertIn("IAD_ABI_VERSION_MINOR is 0", problems[0])
        # a field appended to a sized input at minor 0 is an addition too, that a new minor version covers
        lines = self.baseline.splitlines()
        end = lines.index("struct iad_move_options") + 1
        while lines[end].startswith("    "):
            end += 1
        appended = "\n".join([*lines[:end], "    double appended", *lines[end:]]) + "\n"
        self.assertEqual(self.tool.breaking_changes(self.baseline, appended), [])
        self.assertEqual(len(self.gate(appended)), 1)
        self.assertEqual(self.gate(self.with_minor(appended, 1)), [])

    def test_a_change_to_the_testing_header_does_not_count(self) -> None:
        # no stability promise there: only the stable section is held to the baseline
        text = self.baseline.replace("testing 1\n", "testing 2\n", 1)
        self.assertNotEqual(text, self.baseline)
        self.assertEqual(self.gate(text + "function iad_testing_added void(void)\n"), [])


if __name__ == "__main__":
    unittest.main()
