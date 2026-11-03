#!/usr/bin/env python3
"""One version everywhere: check the release tag against every place the version is written, and print the forms
each package format needs.

    python .github/release/check_version.py --tag v0.12.0        (a tag push: the tag must match the sources)
    python .github/release/check_version.py                      (no tag: the version of bindings/python)

Version format: X.Y.Z, optionally with a pre-release aN, bN or rcN (PEP 440, e.g. 0.12.0a1). The tag is "v" + that.
Sources compared (a missing file is reported and skipped; a version that differs fails the check):

    bindings/python/pyproject.toml        [project] version              (required)
    core/include/iai_actuator_driver/version.hpp   kVersion
    core/CMakeLists.txt                   project(... VERSION X.Y.Z)    (X.Y.Z only: CMake has no pre-release)
    bindings/dotnet/Directory.Build.props <Version>, else every bindings/dotnet/src/*/*.csproj <Version>
                                          (NuGet form allowed: 0.12.0-alpha.1 equals 0.12.0a1)

Outputs (also written to $GITHUB_OUTPUT on GitHub Actions):
    version        0.12.0a1        Python wheels and sdist, file names, the GUI zip, the SDK zips
    nuget_version  0.12.0-alpha.1  NuGet packages (SemVer 2)
    cmake_version  0.12.0          the SDK's CMake package version
    prerelease     true / false    the GitHub release is marked pre-release when true
    tag            v0.12.0a1
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import tomllib
from pathlib import Path

VERSION_RE = re.compile(
    r"^(\d+)\.(\d+)\.(\d+)(?:[-._]?(a|alpha|b|beta|rc|c|pre|preview)[-._]?(\d+))?$", re.IGNORECASE)
PRE_CANON = {"a": "a", "alpha": "a", "b": "b", "beta": "b", "rc": "rc", "c": "rc", "pre": "rc", "preview": "rc"}
NUGET_PRE = {"a": "alpha", "b": "beta", "rc": "rc"}


def parse(text: str) -> tuple[int, int, int, str, int] | None:
    """(major, minor, patch, pre_kind, pre_number); pre_kind "" for a final release. None when not a version."""
    m = VERSION_RE.match(text.strip())
    if not m:
        return None
    pre = PRE_CANON[m.group(4).lower()] if m.group(4) else ""
    return int(m.group(1)), int(m.group(2)), int(m.group(3)), pre, int(m.group(5) or 0)


def pep440(v: tuple[int, int, int, str, int]) -> str:
    return f"{v[0]}.{v[1]}.{v[2]}" + (f"{v[3]}{v[4]}" if v[3] else "")


def nuget(v: tuple[int, int, int, str, int]) -> str:
    return f"{v[0]}.{v[1]}.{v[2]}" + (f"-{NUGET_PRE[v[3]]}.{v[4]}" if v[3] else "")


def read_sources(root: Path) -> tuple[list[tuple[str, str, bool]], list[str]]:
    """[(where, version text, release-only)], notes about missing files."""
    found: list[tuple[str, str, bool]] = []
    notes: list[str] = []

    pyproject = root / "bindings" / "python" / "pyproject.toml"
    if pyproject.is_file():
        data = tomllib.loads(pyproject.read_text(encoding="utf-8"))
        found.append(("bindings/python/pyproject.toml", str(data.get("project", {}).get("version", "")), False))
    else:
        notes.append("bindings/python/pyproject.toml: missing (required)")

    header = root / "core" / "include" / "iai_actuator_driver" / "version.hpp"
    if header.is_file():
        m = re.search(r'kVersion\s*=\s*"([^"]+)"', header.read_text(encoding="utf-8"))
        found.append(("core/include/iai_actuator_driver/version.hpp", m.group(1) if m else "", False))
    else:
        notes.append("core/include/iai_actuator_driver/version.hpp: missing, skipped")

    cmake = root / "core" / "CMakeLists.txt"
    if cmake.is_file():
        m = re.search(r"project\s*\([^)]*?\bVERSION\s+([0-9.]+)", cmake.read_text(encoding="utf-8"), re.S)
        found.append(("core/CMakeLists.txt", m.group(1) if m else "", True))
    else:
        notes.append("core/CMakeLists.txt: missing, skipped")

    dotnet = root / "bindings" / "dotnet"
    props = dotnet / "Directory.Build.props"
    version_tag = re.compile(r"<Version>\s*([^<\s]+)\s*</Version>")
    if props.is_file() and version_tag.search(props.read_text(encoding="utf-8")):
        found.append(("bindings/dotnet/Directory.Build.props",
                      version_tag.search(props.read_text(encoding="utf-8")).group(1), False))
    elif dotnet.is_dir():
        for proj in sorted(dotnet.glob("src/*/*.csproj")):
            m = version_tag.search(proj.read_text(encoding="utf-8"))
            if m:
                found.append((proj.relative_to(root).as_posix(), m.group(1), False))
    else:
        notes.append("bindings/dotnet: missing, skipped")
    return found, notes


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check the release tag against the versions in the sources.")
    parser.add_argument("--tag", default="", help="the pushed tag (vX.Y.Z[aN|bN|rcN]); empty: no tag")
    parser.add_argument("--root", type=Path, default=Path("."))
    args = parser.parse_args(argv)

    found, notes = read_sources(args.root)
    for note in notes:
        print(f"note: {note}")
    if not found or found[0][0] != "bindings/python/pyproject.toml":
        print("error: bindings/python/pyproject.toml has no [project] version")
        return 1

    tag = args.tag.removeprefix("refs/tags/")
    if tag:
        if not tag.startswith("v") or parse(tag[1:]) is None or not re.fullmatch(r"v\d+\.\d+\.\d+((a|b|rc)\d+)?", tag):
            print(f"error: tag {tag!r} is not vX.Y.Z, vX.Y.ZaN, vX.Y.ZbN or vX.Y.ZrcN")
            return 1
        want = parse(tag[1:])
    else:
        want = parse(found[0][1])
        if want is None:
            print(f"error: bindings/python/pyproject.toml version {found[0][1]!r} is not X.Y.Z[aN|bN|rcN]")
            return 1
    assert want is not None

    bad = 0
    for where, text, release_only in found:
        got = parse(text)
        same = got is not None and (got[:3] == want[:3] if release_only else got == want)
        print(f"{'ok      ' if same else 'MISMATCH'} {where}: {text or '(none)'}")
        bad += not same
    if bad:
        print(f"error: {bad} source(s) do not carry version {pep440(want)}"
              + (f" (tag {tag})" if tag else "") + ": set one version everywhere before tagging")
        return 1

    outputs = {
        "version": pep440(want),
        "nuget_version": nuget(want),
        "cmake_version": f"{want[0]}.{want[1]}.{want[2]}",
        "prerelease": "true" if want[3] else "false",
        "tag": tag or f"v{pep440(want)}",
    }
    for key, value in outputs.items():
        print(f"{key}={value}")
    github_output = os.environ.get("GITHUB_OUTPUT")
    if github_output:
        with open(github_output, "a", encoding="utf-8") as fh:
            for key, value in outputs.items():
                fh.write(f"{key}={value}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
