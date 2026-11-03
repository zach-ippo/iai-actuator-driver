"""Command line of the hardware acceptance harness.

    python tools/hw/run.py [run] --port COM3            the acceptance run (the safe motion set)
    python tools/hw/run.py power-cycle --port COM3      you switch the unit's 24 V off and on; the driver recovers it
    python tools/hw/run.py pio-watch --port COM3        you operate the PIO inputs; the harness shows what the unit does

Every subcommand also runs with --sim on the driver's simulator, without a serial port.
Exit codes: 0 no failure, 1 a failed step, 2 refused or could not start (the unit was not changed), 130 Ctrl+C. The
interactive subcommands add their own (see their --help).
"""

from __future__ import annotations

import argparse
import datetime as dt
import sys
from collections.abc import Sequence
from types import SimpleNamespace
from typing import Any

from .session import (EXIT_REFUSED, Console, LogBridge, SetupError, add_connection_arguments, check_connection_arguments, load_driver,
                      require_confirmation_path, results_folder)

SUBCOMMANDS = ("run", "power-cycle", "pio-watch")
EDITIONS = ("python", "csharp", "cli")


def build_run_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="tools/hw/run.py", formatter_class=argparse.RawDescriptionHelpFormatter,
        description="Hardware acceptance run for the IAI actuator driver: a read-only backup first, then the safe motion set inside a "
                    "band of the stroke at a capped speed, then the unit is put back as found. One PASS / CHECK / FAIL / SKIP line "
                    "per step; summary.json and summary.md in the results folder.",
        epilog="Other subcommands: power-cycle (the unit's 24 V is switched by hand), pio-watch (the PIO inputs are operated by "
               "hand); each has its own --help. --sim rehearses everything without a serial port.")
    add_connection_arguments(parser)
    parser.add_argument("--edition", choices=EDITIONS, default="python",
                        help="python: the Python package (default); csharp: the C# command line; cli: the C++ command line. The "
                             "command lines are run as programs and are SKIPped when they cannot be found")
    parser.add_argument("--steps", default="default", metavar="LIST",
                        help="comma list of steps, or 'default' (the safe motion set), or 'all' (adds the opt-in steps); "
                             "'default,recover' adds one. See --list-steps")
    parser.add_argument("--skip", default="", metavar="LIST", help="comma list of steps to leave out")
    parser.add_argument("--list-steps", action="store_true", help="print the steps of the edition and what each checks, then exit")
    parser.add_argument("--sim-scenario", choices=("normal", "alarm", "warning", "cold", "outside", "pio-input", "obstacle", "fault", "interrupt"),
                        default="normal", metavar="NAME",
                        help="--sim only, to rehearse the safety paths: alarm (the unit starts with an alarm: refused), warning (it starts "
                             "with the message-level alarm 0x069, which the recover step resets: the end is CHECK), cold (servo off, "
                             "PIO mode), outside (starts outside the band), pio-input (a PIO input is on), obstacle (something is in the "
                             "push path: contact), fault (the unit stops answering in the middle: a step fails and the unit is put back), interrupt "
                             "(Ctrl+C in the middle of the run)")
    parser.add_argument("--cli-exe", metavar="PATH",
                        help="the command-line program for --edition cli / csharp (default: looked for next to the native core, in the "
                             "build folders and on PATH); a .dll is run with dotnet")
    return parser


def resolve_steps(spec: str, skip: str, all_keys: Sequence[str], default_keys: Sequence[str]) -> list[str]:
    """The selected step keys, in the harness's fixed order."""
    names: set[str] = set()
    for token in (t.strip() for t in spec.split(",") if t.strip()):
        if token == "default":
            names.update(default_keys)
        elif token == "all":
            names.update(all_keys)
        elif token in all_keys:
            names.add(token)
        else:
            raise SetupError(f"unknown step {token!r}; the steps are: {', '.join(all_keys)} (or default, all)")
    for token in (t.strip() for t in skip.split(",") if t.strip()):
        if token not in all_keys:
            raise SetupError(f"--skip {token!r}: not a step; the steps are: {', '.join(all_keys)}")
        names.discard(token)
    return [k for k in all_keys if k in names]


def list_steps(edition: str) -> None:
    if edition == "python":
        from .python_edition import FIXED, STEPS as steps
    else:
        from .cli_edition import FIXED, STEPS as steps
    print(f"Steps of the {edition} edition (the order is fixed; * = not in the default set, M = moves or changes the unit):")
    for key, title in FIXED:
        print(f"  {key:<10}   always  {title}")
    for s in steps:
        print(f"  {s.key:<10} {'*' if not s.default else ' '} {'M' if s.changes else ' '} {s.title}: {s.why}")


def make_context(args: argparse.Namespace, subcommand: str, argv: Sequence[str], load: bool = True) -> SimpleNamespace:
    """Check the arguments, load the driver, make the results folder and log. Raises SetupError before the unit is touched."""
    check_connection_arguments(args)
    require_confirmation_path(args)
    iad = load_driver(args.src, args.native, args.backend) if load else None
    out = results_folder(args.results, args.sim)
    console = Console(out / "run.log")
    ctx = SimpleNamespace(args=args, console=console, out=out, iad=iad, subcommand=subcommand, started=dt.datetime.now(),
                          command=" ".join(["tools/hw/run.py", subcommand, *argv]), unit={}, driver=None,
                          log_bridge=LogBridge(console) if iad is not None else None)
    console.say(f"results folder : {out}")
    if iad is not None:
        info = iad.backend_info()
        ctx.driver = {"package": iad.__version__, "backend": iad.BACKEND, "core": info.get("core_version"), "abi": info.get("abi"),
                      "file": iad.__file__}
        console.say(f"driver         : iai_actuator_driver {iad.__version__}, backend {iad.BACKEND}"
                    + (f", core {info.get('core_version')} (C ABI {info.get('abi')})" if iad.BACKEND == "native" else "") + f"  [{iad.__file__}]")
    ctx.close = lambda: (ctx.log_bridge.detach() if ctx.log_bridge else None, console.close())
    console.say("mode           : " + ("SIMULATOR (--sim): no serial port is opened" if args.sim else f"HARDWARE, port {args.port}, slave {args.slave}"))
    return ctx


def run_command(argv: Sequence[str]) -> int:
    args = build_run_parser().parse_args(list(argv))
    if args.list_steps:
        list_steps(args.edition)
        return 0
    try:
        if args.edition == "python":
            from .python_edition import STEP_KEYS, PythonEdition, default_steps

            selected = resolve_steps(args.steps, args.skip, STEP_KEYS, default_steps())
            ctx = make_context(args, "run", argv)
            edition: Any = PythonEdition(ctx, selected)
        else:
            from .cli_edition import STEP_KEYS, CliEdition, default_steps

            selected = resolve_steps(args.steps, args.skip, STEP_KEYS, default_steps())
            ctx = make_context(args, "run", argv, load=False)
            edition = CliEdition(ctx, selected, args.edition)
    except SetupError as exc:
        print(f"REFUSED (exit {EXIT_REFUSED}): {exc}", file=sys.stderr)
        return EXIT_REFUSED
    try:
        ctx.console.say(f"edition        : {args.edition}; steps: {', '.join(selected)}")
        return int(edition.run())
    finally:
        ctx.close()


def power_cycle_command(argv: Sequence[str]) -> int:
    from . import power_cycle

    return power_cycle.main(argv)


def pio_watch_command(argv: Sequence[str]) -> int:
    from . import pio_watch

    return pio_watch.main(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if args and args[0] in SUBCOMMANDS:
        command, rest = args[0], args[1:]
    else:
        command, rest = "run", args
    try:
        if command == "power-cycle":
            return power_cycle_command(rest)
        if command == "pio-watch":
            return pio_watch_command(rest)
        return run_command(rest)
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        return 130

