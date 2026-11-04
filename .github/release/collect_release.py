#!/usr/bin/env python3
"""Gather the release assets into one folder and write the release notes.

    python .github/release/collect_release.py --artifacts artifacts --out release --version X.Y.Z --tag vX.Y.Z
    python .github/release/collect_release.py ... --optional GUI     (no GUI built: no apps/gui/iai-gui.spec)
    python .github/release/collect_release.py ... --minimum "Python wheels=1" --note "Windows x64 only."
                                                          (a release built for fewer platforms: local_release.py)

Every file under --artifacts that is a release asset (wheel, sdist, nupkg, the SDK zips, the GUI
zip) is copied flat into --out. Two asset names must never collide; a missing kind of asset fails the step, so a
release is never published half built, unless that kind is named with --optional (then the notes say it is not in
this release). An asset without the repository's LICENSE, NOTICE and NATIVE_THIRD_PARTY.txt (check_package.NOTICES,
missing_notices) fails it too.
The notes are the CHANGELOG.md section of the version (a "## " heading that names it) followed by a table of the assets.
"""

from __future__ import annotations

import argparse
import re
import shutil
import sys
from pathlib import Path

from check_package import NOTICES, missing_notices  # this folder: the script's own, or on the path of local_release.py

KINDS = [
    # (label, glob, how many at least, what it is)
    ("Python wheels", "*.whl", 2, "`pip install` this file: the Python package with the native library of that platform"),
    ("Python sdist", "*.tar.gz", 1, "source distribution: needs the native library, which it does not carry"),
    ("NuGet packages", "*.nupkg", 1, "C# package IaiActuatorDriver, with the native library under `runtimes/<rid>/native`"),
    ("C/C++ SDK", "iai_actuator_driver-sdk-*.zip", 2, "header, shared library, import library, CMake config"),
    ("GUI", "iai-gui-*.zip", 1, "iai-gui.exe (Windows x64, Python inside), README.txt and the licence files"),
]


def changelog_section(changelog: Path, version: str) -> str:
    if not changelog.is_file():
        return ""
    lines = changelog.read_text(encoding="utf-8").splitlines()
    start = None
    token = re.compile(rf"(^|[\s\[v]){re.escape(version)}([\s\]),]|$)")
    for i, line in enumerate(lines):
        if line.startswith("## ") and token.search(line[3:]):
            start = i + 1
            break
    if start is None:
        return ""
    end = next((j for j in range(start, len(lines)) if lines[j].startswith("## ")), len(lines))
    return "\n".join(lines[start:end]).strip()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Gather the release assets and notes.")
    parser.add_argument("--artifacts", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--version", required=True)
    parser.add_argument("--tag", required=True)
    parser.add_argument("--changelog", type=Path, default=Path("CHANGELOG.md"))
    parser.add_argument("--optional", action="append", default=[], choices=[k[0] for k in KINDS], metavar="KIND",
                        help="a kind of asset this release may lack (e.g. GUI); repeatable")
    parser.add_argument("--minimum", action="append", default=[], metavar="KIND=N",
                        help="fewer files of a kind than the default, e.g. 'Python wheels=1' for a release built for "
                             "fewer platforms (local_release.py); repeatable")
    parser.add_argument("--note", action="append", default=[], metavar="TEXT",
                        help="a paragraph for the notes, after the asset table; repeatable")
    args = parser.parse_args(argv)

    minimum = {}
    for spec in args.minimum:
        kind, _, count = spec.rpartition("=")
        if kind not in [k[0] for k in KINDS] or not count.isdigit():
            parser.error(f"--minimum {spec!r}: expected KIND=N with KIND one of {[k[0] for k in KINDS]}")
        minimum[kind] = int(count)

    args.out.mkdir(parents=True, exist_ok=True)
    errors = []
    table = []
    missing = []
    for label, pattern, at_least, what in KINDS:
        at_least = minimum.get(label, at_least)
        files = sorted(p for p in args.artifacts.rglob(pattern) if p.is_file())
        if not files and label in args.optional:
            missing.append(label)
            continue
        if len(files) < at_least:
            errors.append(f"{label}: {len(files)} file(s) matching {pattern}, expected at least {at_least}")
        for f in files:
            lacking = missing_notices(f)
            if lacking:
                errors.append(f"{f.name} lacks {', '.join(lacking)}: every asset carries {', '.join(NOTICES)}")
            dest = args.out / f.name
            if dest.exists():
                errors.append(f"two assets named {f.name}")
                continue
            shutil.copy2(f, dest)
            table.append((label, f.name, what))
    if errors:
        for e in errors:
            print(f"error: {e}")
        return 1

    section = changelog_section(args.changelog, args.version)
    notes = [f"IAI Actuator Driver {args.version}. Unofficial: not an IAI product.", ""]
    notes += [section, ""] if section else [f"See CHANGELOG.md at {args.tag} for the changes.", ""]
    notes += ["## Assets", "", "| Kind | File | What |", "|---|---|---|"]
    notes += [f"| {label} | `{name}` | {what} |" for label, name, what in table]
    if missing:
        notes += ["", f"Not in this release: {', '.join(missing)}."]
    for note in args.note:
        notes += ["", note]
    notes_file = args.out.parent / "release-notes.md"
    notes_file.write_text("\n".join(notes) + "\n", encoding="utf-8", newline="\n")

    print(f"{len(table)} asset(s) in {args.out}, notes in {notes_file}")
    for label in missing:
        print(f"  not in this release (--optional): {label}")
    for _, name, _ in table:
        print(f"  {name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
