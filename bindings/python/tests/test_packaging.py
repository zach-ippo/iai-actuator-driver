"""Packaging checks: one version everywhere, console scripts, shipped GUI files, documentation links and wording.

No hardware and no serial port: this reads files only.
"""

from __future__ import annotations

import re
import tomllib
import unittest
from collections.abc import Iterator
from pathlib import Path

import iai_actuator_driver

PKG = Path(__file__).resolve().parent.parent  # bindings/python: the project folder of the driver
ROOT = PKG.parents[1]  # the repository root
GUI = ROOT / "apps" / "gui"  # the GUI and its own pyproject.toml
GITHUB = "https://github.com/zach-ippo/iai-actuator-driver/"
DOCS = (
    "README.md", "CHANGELOG.md", "docs/GETTING_STARTED.md", "docs/COMPATIBILITY.md", "docs/DRIVER.md", "docs/REFERENCE.md",
    "docs/HARDWARE_SETUP.md", "docs/TROUBLESHOOTING.md", "docs/FAQ.md", "docs/ARCHITECTURE.md", "examples/README.md",
    ".github/CONTRIBUTING.md", "docs/PYTHON_FALLBACK.md", ".github/SECURITY.md",
)
# the licence files the packages carry, and where the repository keeps each: LICENSE at the root (where GitHub finds the
# licence), the notices in licenses/ (.github/release/check_package.py NOTICES)
LICENCE_FILES = {
    "LICENSE": "LICENSE", "NOTICE": "licenses/NOTICE", "NATIVE_THIRD_PARTY.txt": "licenses/NATIVE_THIRD_PARTY.txt",
}
# Words that must never appear in anything shipped (built by concatenation so this file does not trip itself).
FORBIDDEN = ("il" + "spy", "decom" + "pil", "dn" + "spy", "research" + "/", "research" + "\\", "internal " + "research")
TEXT_SUFFIXES = {".py", ".md", ".toml", ".js", ".html", ".css", ".json", ".txt", ".cfg", ".gitignore"}


def pyproject() -> dict:
    return tomllib.loads((PKG / "pyproject.toml").read_text(encoding="utf-8"))


def gui_pyproject() -> dict:
    return tomllib.loads((GUI / "pyproject.toml").read_text(encoding="utf-8"))


class VersionAndEntryPointsTest(unittest.TestCase):
    def test_one_version_everywhere(self) -> None:
        version = pyproject()["project"]["version"]
        self.assertEqual(version, "1.1.1")
        self.assertEqual(iai_actuator_driver.__version__, version)
        gui = gui_pyproject()["project"]
        self.assertEqual(gui["version"], version)
        self.assertEqual(gui["dependencies"], [f"iai-actuator-driver=={version}"])
        changelog = (ROOT / "CHANGELOG.md").read_text(encoding="utf-8")
        # an "## Unreleased" section above the version heading is allowed: it is what the next release will say
        newest = re.search(r"^## (?!Unreleased\b)(\S+)", changelog, re.M)
        self.assertIsNotNone(newest)
        self.assertEqual(newest.group(1), version)
        self.assertIn(version, (ROOT / "README.md").read_text(encoding="utf-8"))

    def test_the_release_version_check_passes(self) -> None:
        """.github/release/check_version.py: this package, the core and the C# projects carry one version."""
        import contextlib
        import importlib.util
        import io
        spec = importlib.util.spec_from_file_location("check_version", ROOT / ".github" / "release" / "check_version.py")
        self.assertIsNotNone(spec)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = module.main(["--root", str(ROOT)])
        self.assertEqual(code, 0, out.getvalue())
        self.assertIn(f"version={pyproject()['project']['version']}", out.getvalue())

    def test_release_metadata(self) -> None:
        project = pyproject()["project"]
        self.assertIn("Development Status :: 5 - Production/Stable", project["classifiers"])
        self.assertNotIn("Development Status :: 4 - Beta", project["classifiers"])
        self.assertIn("UNOFFICIAL", project["description"])
        self.assertIn("not an iai product", project["description"].lower())
        self.assertEqual(project["readme"], "README.md")
        self.assertEqual(project["dependencies"], ["pyserial>=3.5"])

    def test_console_scripts(self) -> None:
        scripts = pyproject()["project"]["scripts"]
        self.assertEqual(scripts, {
            "iai-actuator-driver": "iai_actuator_driver.cli:main", "iai-check": "iai_actuator_driver.cli:check_main"})
        self.assertEqual(gui_pyproject()["project"]["scripts"], {"iai-gui": "gui.server:main"})
        self.assertIn("def main(", (GUI / "server.py").read_text(encoding="utf-8"))
        import importlib
        for target in scripts.values():
            module, _, name = target.partition(":")
            self.assertTrue(callable(getattr(importlib.import_module(module), name)), target)

    def test_the_gui_is_packaged_with_its_pages(self) -> None:
        self.assertIn("iai_actuator_driver", pyproject()["tool"]["setuptools"]["packages"])
        tool = gui_pyproject()["tool"]["setuptools"]
        self.assertEqual(tool["packages"], ["gui"])
        self.assertEqual(tool["package-dir"], {"gui": "."})
        patterns = tool["package-data"]["gui"]
        for page in ("workbench.html", "workbench.js", "workbench.css", "index.html", "app.js", "about.js", "styles.css",
                     "coord.js", "coord.css"):
            self.assertTrue(any(Path(page).match(p) for p in patterns), page)
            self.assertTrue((GUI / page).is_file(), page)


class LicenceTest(unittest.TestCase):
    def test_the_package_copies_are_the_root_files(self) -> None:
        """A pyproject.toml cannot name files outside its folder, so bindings/python and apps/gui hold copies of LICENSE
        and NOTICE for their license-files, and bindings/python one of NATIVE_THIRD_PARTY.txt too (its wheels carry the
        native library; the GUI package is pure Python): copy the repository's file again when this fails."""
        for name, folders in (("LICENSE", (PKG, GUI)), ("NOTICE", (PKG, GUI)), ("NATIVE_THIRD_PARTY.txt", (PKG,))):
            for folder in folders:
                copy = folder / name
                self.assertTrue(copy.is_file(), copy)
                own = LICENCE_FILES[name]
                self.assertEqual(copy.read_bytes(), (ROOT / own).read_bytes(), f"{copy} differs from {own}")

    def test_licence_metadata_and_notice(self) -> None:
        for project, files in ((pyproject()["project"], ["LICENSE", "NOTICE", "NATIVE_THIRD_PARTY.txt"]),
                               (gui_pyproject()["project"], ["LICENSE", "NOTICE"])):
            self.assertEqual(project["license"], "Apache-2.0")
            self.assertEqual(project["license-files"], files)
            self.assertEqual(project["authors"], [{"name": "Zachary Ippolito"}])
            self.assertFalse([c for c in project["classifiers"] if c.startswith("License ::")])
        self.assertIn("Apache License", (ROOT / "LICENSE").read_text(encoding="utf-8")[:200])
        notice = (ROOT / LICENCE_FILES["NOTICE"]).read_text(encoding="utf-8")
        self.assertTrue(notice.startswith("IAI Actuator Driver\nCopyright 2026 Zachary Ippolito\n"), notice)
        self.assertIn("not an IAI product and is not supported, documented or endorsed by IAI", " ".join(notice.split()))
        self.assertIn("RoboCylinder", notice)  # the spelling of every other file


class DocumentationTest(unittest.TestCase):
    def test_every_document_exists_and_the_readme_links_to_them(self) -> None:
        for rel in DOCS:
            self.assertTrue((ROOT / rel).is_file(), rel)
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        for rel in DOCS[1:]:
            self.assertIn(rel, readme, rel)

    def test_relative_links_resolve(self) -> None:
        for rel in DOCS:
            path = ROOT / rel
            for target in re.findall(r"\]\(([^)#\s]+)(?:#[^)]*)?\)", path.read_text(encoding="utf-8")):
                if re.match(r"^[a-z]+:", target):
                    continue
                self.assertTrue((path.parent / target).exists(), f"{rel} links to missing {target}")

    def test_the_package_readmes_link_to_github_not_to_files(self) -> None:
        """bindings/python/README.md is the wheel's description, bindings/dotnet/README.md ships inside the NuGet package
        and apps/gui/README.md is the GUI wheel's description: a relative link does not work in any of them."""
        for rel in ("bindings/python/README.md", "bindings/dotnet/README.md", "apps/gui/README.md"):
            for target in re.findall(r"\]\(([^)#\s]+)(?:#[^)]*)?\)", (ROOT / rel).read_text(encoding="utf-8")):
                self.assertTrue(target.startswith("https://"), f"{rel} has a relative link: {target}")
                if target.startswith(GITHUB + "blob/main/"):
                    self.assertTrue((ROOT / target.removeprefix(GITHUB + "blob/main/")).exists(), f"{rel}: {target}")

    def test_readme_is_a_front_page_with_a_ten_line_quick_start(self) -> None:
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        self.assertLess(len(readme.splitlines()), 160)
        for heading in ("## What it does", "## Install", "## Quick start", "## Which units work", "## Documentation"):
            self.assertIn(heading, readme)
        self.assertIn("UNOFFICIAL", readme)
        self.assertNotIn("UNOFFICIAL BETA", readme)
        self.assertIn("not an IAI product", readme)
        block = re.search(r"## Quick start.*?```python\n(.*?)```", readme, re.S)
        self.assertIsNotNone(block)
        self.assertLessEqual(len(block.group(1).rstrip().splitlines()), 10)

    def test_compatibility_table_has_every_group(self) -> None:
        text = (ROOT / "docs" / "COMPATIBILITY.md").read_text(encoding="utf-8")
        for needle in ("## Run on hardware so far", "## Notes per product line", "## Not supported",
                       "## Possible trouble", "Every IAI unit", "driven the same way",
                       "PCON-CB with RCP6", "ACON-CB, DCON-CB, SCON-CB", "RCON driver units", "ROBONET", "SW2",
                       "XSEL", "SSEL", "ASEL", "PSEL", "MSEL", "TTA", "RCON-EC", "REC", "EC 200 V",
                       "baud", "line break", "teaching pendant", "wireless", "register directory", "CCM conveyor",
                       "Vertical axes with a brake"):
            self.assertIn(needle, text, needle)

    def test_getting_started_covers_the_first_run(self) -> None:
        text = (ROOT / "docs" / "GETTING_STARTED.md").read_text(encoding="utf-8")
        for needle in ("Close IA-OS", "iai-actuator-driver scan", "iai-gui --sim-only", "first move", "## Safety", "iai-check"):
            self.assertIn(needle.lower(), text.lower(), needle)

    def test_documented_commands_exist(self) -> None:
        """Every ``iai-actuator-driver ...`` sub-command in the README and the getting-started guide is a real one."""
        from iai_actuator_driver.cli import PORT_COMMANDS, build_parser
        names = set(build_parser()._subparsers._group_actions[0].choices) | set(PORT_COMMANDS)  # noqa: SLF001
        for rel in ("README.md", "docs/GETTING_STARTED.md", "docs/HARDWARE_SETUP.md"):
            text = (ROOT / rel).read_text(encoding="utf-8")
            for match in re.finditer(r"^iai-actuator-driver (?:--port \S+ |--sim )?([a-z][a-z-]*)", text, re.M):
                self.assertIn(match.group(1), names, f"{rel}: {match.group(0)}")


def _shipped_texts() -> Iterator[tuple[Path, str]]:
    """Every shipped text file (this test file excluded) with its text."""
    skip = {Path(__file__).resolve()}
    for path in ROOT.rglob("*"):
        if not path.is_file() or path.resolve() in skip:
            continue
        parts = set(path.relative_to(ROOT).parts)
        if parts & {".git", "__pycache__", "backups", ".claude", ".venv", "venv", "build", "dist"}:
            continue
        if path.suffix.lower() not in TEXT_SUFFIXES and path.name != ".gitignore":
            continue
        yield path, path.read_text(encoding="utf-8", errors="replace")


class ShippedWordingTest(unittest.TestCase):
    def test_no_forbidden_words_in_any_shipped_text(self) -> None:
        hits = []
        for path, text in _shipped_texts():
            for word in FORBIDDEN:
                if word in text.lower():
                    hits.append(f"{path.relative_to(ROOT)}: {word!r}")
        self.assertEqual(hits, [])

    def test_units_are_named_by_model_not_by_number(self) -> None:
        hits = [f"{path.relative_to(ROOT)}: {m.group()!r}" for path, text in _shipped_texts()
                for m in re.finditer(r"\bEC #\d", text)]
        self.assertEqual(hits, [])

    def test_docs_use_the_four_evidence_words(self) -> None:
        for rel in ("README.md", "docs/COMPATIBILITY.md", "docs/REFERENCE.md"):
            text = (ROOT / rel).read_text(encoding="utf-8")
            for word in ("VERIFIED", "ON-WIRE", "DOC", "UNVERIFIED"):
                self.assertIn(word, text, f"{rel}: {word}")


if __name__ == "__main__":
    unittest.main()
