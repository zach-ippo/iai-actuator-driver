#!/usr/bin/env python3
"""Run ec_tests as several processes at once and merge their output into one summary.

    python core/tests/run_shards.py [--shards N] [--quiet] [--timeout S] [--cwd DIR] [--tail NAME]... EXE [PREFIX ...]

The tests mostly wait on the simulator's serial frame timing, so they run in parallel processes almost as fast as one
alone. Each process runs ``EXE --shard K/N [PREFIX ...]`` (every N-th of the selected tests, in their usual order) and
has its own temp directory (test_main.cpp). The runner:

* lists the selection first (``EXE -l [PREFIX ...]``) and fails when no test matches;
* runs N shards at once (default: half the CPUs, at most 6; never more than the selected tests); ``--shards 1`` runs
  the executable directly, its output as it comes, and holds it to ``--timeout`` too;
* prints one line per shard as it ends, and the whole output of a shard that failed, crashed, was killed by a signal,
  ended without its summary or ran past ``--timeout`` seconds (default 1800; 0: no limit); a shard that ended without
  its summary is named with the test it was running;
* runs the selected tests named by ``--tail`` (the load-sensitive tests of tools/load_sensitive.txt, which
  tools/test.py passes) not in the shards (``EXE --exclude NAME``) but after them, one after another in one more process
  (``EXE --exact NAME ...``), and the tail's failed tests once more in another: their second outcome counts, and the
  summary names a test that passed then (``passed on its retry: NAME``);
* checks that the shards and the tail together ran every selected test exactly once;
* prints the merged summary in the executable's own format (``N tests, P passed, F failed, C checks, T s`` and a
  ``failed:`` line per failed test; the tests a crashed or killed shard reported before it ended count, so the totals
  and the tests that did not run add up to the selection). Exit code 0 only when every shard passed and no test was
  lost or run twice.

No test opens a serial port: every test runs on the simulator.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import tempfile
import time
from collections import Counter
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import IO

MAX_DEFAULT_SHARDS = 6
DEFAULT_TIMEOUT_S = 1800.0
_LINE = re.compile(r"^  (ok  |FAIL) (\S+)\s+[\d.]+ s$")
_SUMMARY = re.compile(r"^(\d+) tests, (\d+) passed, (\d+) failed, (\d+) checks, [\d.]+ s$")


def default_shards() -> int:
    """About half the CPUs, at most 6: the tests wait on the simulator clock, so more buys little and loads the PC."""
    return max(1, min(MAX_DEFAULT_SHARDS, (os.cpu_count() or 2) // 2))


@dataclass
class Shard:
    number: int  # 1-based
    count: int
    title: str = ""  # the label of a process that is not one of the shards (the tail, its retry)
    proc: subprocess.Popen | None = None
    log: IO[bytes] | None = None
    started: float = 0.0
    seconds: float = 0.0
    timed_out: bool = False
    code: int | None = None
    text: str = ""
    names: list[str] = field(default_factory=list)  # the tests it reported, in order
    failed: list[str] = field(default_factory=list)
    selection: list[str] = field(default_factory=list)  # the tests it was to run, in order
    summary: tuple[int, int, int, int] | None = None  # run, passed, failed, checks

    @property
    def label(self) -> str:
        return self.title or f"shard {self.number}/{self.count}"

    def running(self) -> str:
        """`` while running <test>``: the first test of its selection that printed no line (it runs them in order);
        empty when every test reported or the selection is not known."""
        reported = set(self.names)
        name = next((n for n in self.selection if n not in reported), None)
        return f" while running {name}" if name else ""

    def problem(self) -> str | None:
        """Why this shard failed, or None when it passed."""
        if self.timed_out:
            return f"killed after the time limit ({self.seconds:.0f} s){self.running()}"
        if self.code is None:
            return "did not run"
        if self.summary is None:
            return f"ended without a summary ({describe_exit(self.code)}){self.running()}"
        if self.code == 1 and self.summary[2]:
            return None  # failed tests: listed in the merged summary
        if self.code != 0:
            return describe_exit(self.code)  # e.g. memory errors (EC_TESTS_POISON) after a clean summary
        return None


def describe_exit(code: int) -> str:
    if code < 0:
        return f"killed by signal {-code}"
    if code > 255:  # a Windows exception code (0xC0000005: access violation, ...)
        return f"crashed (exit code 0x{code & 0xFFFFFFFF:08X})"
    return f"exit code {code}"


def list_tests(exe: Path, prefixes: Sequence[str], cwd: Path) -> list[str]:
    proc = subprocess.run([str(exe), "-l", *prefixes], cwd=cwd, capture_output=True)
    if proc.returncode != 0:
        sys.stdout.write(proc.stdout.decode("utf-8", "replace") + proc.stderr.decode("utf-8", "replace"))
        raise SystemExit(f"{exe.name} -l failed ({describe_exit(proc.returncode)})")
    return [line.strip() for line in proc.stdout.decode("utf-8", "replace").splitlines() if line.strip()]


def parse(shard: Shard) -> None:
    for line in shard.text.splitlines():
        line = line.rstrip("\r")
        m = _LINE.match(line)
        if m:
            shard.names.append(m.group(2))
            if m.group(1) == "FAIL":
                shard.failed.append(m.group(2))
            continue
        m = _SUMMARY.match(line)
        if m:
            shard.summary = (int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4)))


def run_direct(exe: Path, prefixes: Sequence[str], quiet: bool, timeout_s: float, cwd: Path, out: IO[str]) -> int:
    """The executable alone, its output going straight to this process's own; killed after ``timeout_s``."""
    proc = subprocess.Popen([str(exe), *(["--quiet"] if quiet else []), *prefixes], cwd=cwd)
    try:
        return proc.wait(timeout=timeout_s if timeout_s > 0 else None)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        print(f"\n{exe.name}: killed after the time limit ({timeout_s:.0f} s)", file=out, flush=True)
        return 1
    except BaseException:  # Ctrl+C: leave no test process behind
        proc.kill()
        proc.wait()
        raise


def run(exe: Path | str, prefixes: Sequence[str] = (), shards: int | None = None, quiet: bool = False,
        timeout_s: float = DEFAULT_TIMEOUT_S, cwd: Path | str | None = None, out: IO[str] | None = None,
        tail: Sequence[str] = ()) -> int:  # fmt: skip
    """Run the selected tests of ``exe`` in ``shards`` processes, those named in ``tail`` after them (with one retry);
    returns the exit code (0: all passed)."""
    out = out or sys.stdout
    exe = Path(exe).resolve()
    cwd = Path(cwd) if cwd else Path.cwd()
    prefixes = list(prefixes)
    requested = default_shards() if shards is None else max(1, shards)
    if requested == 1:
        out.flush()
        return run_direct(exe, prefixes, quiet, timeout_s, cwd, out)

    selected = list_tests(exe, prefixes, cwd)
    if not selected:
        print("\n0 tests, 0 passed, 0 failed, 0 checks, 0.00 s\nno test matched", file=out)
        return 1
    after = set(tail)
    last = [name for name in selected if name in after]  # the serial tail, in the usual order
    expected = [name for name in selected if name not in after]
    count = min(requested, len(expected))
    print(f"{len(expected)} tests in {count} shards (--shards 1: one process)"
          + (f", then {len(last)} alone (load-sensitive)" if last else ""), file=out, flush=True)  # fmt: skip
    excluded = [arg for name in last for arg in ("--exclude", name)]

    started = time.monotonic()
    running: list[Shard] = []
    for k in range(1, count + 1):
        shard = Shard(k, count, selection=expected[k - 1::count])  # as ec_tests --shard K/N: every N-th, from the K-th
        shard.log = tempfile.TemporaryFile()
        shard.started = time.monotonic()
        # never --quiet: every test's line names it, for the check that each test ran exactly once
        shard.proc = subprocess.Popen([str(exe), "--shard", f"{k}/{count}", *excluded, *prefixes], cwd=cwd,
                                      stdout=shard.log, stderr=subprocess.STDOUT)
        running.append(shard)

    done: list[Shard] = []
    try:
        while running:
            for shard in list(running):
                assert shard.proc is not None and shard.log is not None
                code = shard.proc.poll()
                elapsed = time.monotonic() - shard.started
                if code is None and timeout_s > 0 and elapsed > timeout_s:
                    shard.proc.kill()
                    code = shard.proc.wait()
                    shard.timed_out = True
                if code is None:
                    continue
                shard.code, shard.seconds = code, elapsed
                shard.log.seek(0)
                shard.text = shard.log.read().decode("utf-8", "replace").replace("\r\n", "\n")
                shard.log.close()
                parse(shard)
                running.remove(shard)
                done.append(shard)
                report(shard, quiet, out)
            if running:
                time.sleep(0.05)
    finally:
        for shard in running:  # Ctrl+C or an error here: leave no test process behind
            if shard.proc is not None and shard.proc.poll() is None:
                shard.proc.kill()

    retry = None
    if last:
        first = run_alone(exe, last, Shard(count + 1, count, "tail"), timeout_s, cwd)
        report(first, quiet, out)
        done.append(first)
        again = [name for name in last if name in first.failed]
        if again:
            retry = run_alone(exe, again, Shard(count + 2, count, "retry"), timeout_s, cwd)
            report(retry, quiet, out)
    return merge(done, selected, time.monotonic() - started, out, retry)


def run_alone(exe: Path, names: Sequence[str], shard: Shard, timeout_s: float, cwd: Path) -> Shard:
    """``names`` one after another in one process (``EXE --exact NAME ...``), held to ``timeout_s``."""
    shard.selection = list(names)
    shard.started = time.monotonic()
    with tempfile.TemporaryFile() as log:
        proc = subprocess.Popen([str(exe), "--exact", *names], cwd=cwd, stdout=log, stderr=subprocess.STDOUT)
        try:
            shard.code = proc.wait(timeout=timeout_s if timeout_s > 0 else None)
        except subprocess.TimeoutExpired:
            proc.kill()
            shard.code = proc.wait()
            shard.timed_out = True
        except BaseException:  # Ctrl+C: leave no test process behind
            proc.kill()
            proc.wait()
            raise
        shard.seconds = time.monotonic() - shard.started
        log.seek(0)
        shard.text = log.read().decode("utf-8", "replace").replace("\r\n", "\n")
    parse(shard)
    return shard


def report(shard: Shard, quiet: bool, out: IO[str]) -> None:
    problem = shard.problem()
    if shard.summary is not None:
        run_, passed, failed, _ = shard.summary
        head = f"{shard.label}: {run_} tests, {passed} passed, {failed} failed ({shard.seconds:.1f} s)"
    else:
        head = f"{shard.label}: {len(shard.names)} tests reported ({shard.seconds:.1f} s)"
    if problem:
        head += f": {problem}"
    failed = problem or (shard.summary and shard.summary[2])
    if failed or not quiet:
        lines = shard.text.rstrip("\n").splitlines()
        if quiet:  # a failed shard's output, without the lines of its passed tests
            lines = [line for line in lines if not line.startswith("  ok   ")]
        elif not failed:  # its tests; the merged summary follows at the end
            lines = [line for line in lines if line.strip() and not _SUMMARY.match(line)
                     and line != "no test in this shard"]
        print(f"=== {head}", file=out)
        for line in lines:
            print(line, file=out)
        print(f"=== end of {shard.label}", file=out, flush=True)
    else:
        print(head, file=out, flush=True)


def merge(shards: list[Shard], expected: list[str], seconds: float, out: IO[str], retry: Shard | None = None) -> int:
    """The summary of ``shards`` (the tail last); ``retry``: the second run of the tail's failed tests, whose outcome
    replaces the first."""
    run_ = passed = failed = checks = 0
    failed_names: list[str] = []
    seen: Counter[str] = Counter()
    for shard in sorted(shards, key=lambda s: s.number):
        seen.update(shard.names)
        failed_names += shard.failed
        if shard.summary is not None:
            run_ += shard.summary[0]
            passed += shard.summary[1]
            failed += shard.summary[2]
            checks += shard.summary[3]
        else:  # crashed or killed: the tests that reported before it ended still count (its checks are not known)
            run_ += len(shard.names)
            failed += len(shard.failed)
            passed += len(shard.names) - len(shard.failed)
    second = [name for name in retry.names if name not in retry.failed] if retry is not None else []
    if retry is not None:
        checks += retry.summary[3] if retry.summary is not None else 0
        passed, failed = passed + len(second), failed - len(second)
        failed_names = [name for name in failed_names if name not in second]
    wanted = set(expected)
    lost = [name for name in expected if name not in seen]
    twice = sorted(name for name, n in seen.items() if n > 1)
    extra = sorted(name for name in seen if name not in wanted)
    print(f"\n{run_} tests, {passed} passed, {failed} failed, {checks} checks, {seconds:.2f} s", file=out)
    for name in failed_names:
        print(f"  failed: {name}", file=out)
    for name in second:
        print(f"  passed on its retry: {name}", file=out)
    every = sorted(shards, key=lambda s: s.number) + ([retry] if retry is not None else [])
    problems = [f"{s.label}: {s.problem()}" for s in every if s.problem()]
    if lost:
        problems.append(f"{len(lost)} of {len(expected)} selected tests did not run: " + ", ".join(lost[:20])
                        + (" ..." if len(lost) > 20 else ""))
    if twice:
        problems.append("ran more than once: " + ", ".join(twice))
    if extra:
        problems.append("ran but not selected: " + ", ".join(extra))
    for problem in problems:
        print(f"  {problem}", file=out)
    ok = not problems and failed == 0 and run_ == len(expected) > 0
    count = len([s for s in shards if not s.title])
    print(f"{count} shards{' and the tail' if count < len(shards) else ''}: {'OK' if ok else 'FAILED'}", file=out,
          flush=True)  # fmt: skip
    return 0 if ok else 1


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="run_shards.py", description="Run ec_tests in parallel shards.")
    parser.add_argument("exe", help="the test executable (ec_tests)")
    parser.add_argument("prefixes", nargs="*", help="test-name prefixes (default: every test)")
    parser.add_argument("-j", "--shards", type=int, default=None,
                        help=f"processes at once (default {default_shards()}: half the CPUs, at most "
                             f"{MAX_DEFAULT_SHARDS}; 1 runs the executable directly)")
    parser.add_argument("-q", "--quiet", action="store_true", help="no line per passed test")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                        help=f"seconds per shard (with --shards 1: for the run) before it is killed (default "
                             f"{DEFAULT_TIMEOUT_S:g}; 0: no limit)")
    parser.add_argument("--cwd", help="working folder of the tests (default: this one)")
    parser.add_argument("--tail", action="append", default=[], metavar="NAME",
                        help="a test to run after the shards, with the other --tail tests one after another, and once "
                             "more when it failed (the load-sensitive tests; not with --shards 1)")  # fmt: skip
    args = parser.parse_args(argv)
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except (AttributeError, OSError):
        pass
    return run(args.exe, args.prefixes, args.shards, args.quiet, args.timeout, args.cwd, tail=args.tail)


if __name__ == "__main__":
    sys.exit(main())
