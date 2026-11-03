"""Parameters and backup: save a read-only backup, read the parameters, and see the guard on non-volatile writes.

``iai_actuator_driver.info.backup(ax, path)`` saves one unit as JSON (identity, directory, every parameter item, the stored rows,
the alarm history, the maintenance counters) using FC03 / FC11 reads only: nothing is written to the unit. Back a unit
up before any non-volatile write. ``ax.parameters()`` lists the unit's named parameters (the P-numbers of an
ELECYLINDER, the numbered parameters of a CON-family controller) and ``ax.read_parameter(n)`` reads one; both are
read-only.

``ax.write_parameter(n, value)`` writes a parameter, reads it back and commits it to non-volatile memory (the unit
applies it after a software reset or a power cycle). That memory lasts about 100,000 write cycles (IAI), so a value
the unit already holds is neither written nor committed (``result.skipped``); ``force=True`` commits it anyway. Each
axis object counts its non-volatile writes (``ax.nonvolatile_writes``) and logs one WARNING per hour while more than
60 fall in the last hour; a write is never refused for the rate. Direct moves (``move_to``) are RAM only (VERIFIED)
and cost no write cycle.

Run it from the project folder (close IA-OS first: only one program can use the serial port)::

    python examples/python/parameters_and_backup.py --sim                    # simulated ELECYLINDER slider
    python examples/python/parameters_and_backup.py --sim --force            # also the forced commit
    python examples/python/parameters_and_backup.py --sim --sim-profile pcon # a PCON: CON parameter numbers
    python examples/python/parameters_and_backup.py --port COM3
    python examples/python/parameters_and_backup.py --port COM3 --param 2 --backup unit1.json

Writes: the guarded write sends the value the parameter already holds, so the unit's configuration does not change.
Without ``--force`` nothing reaches the non-volatile memory. With ``--force`` that value is committed once: one write
cycle, and the commit also saves any other parameter value still waiting in the unit's RAM (a RAM-only write by
another program, for example). The backup goes to ``backups/`` in the current folder unless ``--backup`` names a
file. Nothing moves and the servo is not touched; Modbus control, if the forced commit had to take it, is handed back
to PIO when PIO had it at the start.

Unofficial: not an IAI product. The driver is not a safety function: keep a hardware emergency stop.
"""

from __future__ import annotations

import argparse
import datetime as dt
import logging
import signal
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2] / "bindings" / "python"
if (ROOT / "iai_actuator_driver").is_dir() and str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))  # a source checkout: use the driver in bindings/python

from iai_actuator_driver import Axis, EcDriverError, ParameterWriteResult, SimulatedEC  # noqa: E402
from iai_actuator_driver.ec import NV_WARN_PER_HOUR  # noqa: E402
from iai_actuator_driver.info import backup  # noqa: E402
from iai_actuator_driver.sim import PROFILES  # noqa: E402

BANNER = "iai-actuator-driver example: parameters and backup (UNOFFICIAL - not an IAI product)"


def build_parser() -> argparse.ArgumentParser:
    """Command-line options: ``--port COM3`` or ``--sim`` (one of them is required)."""
    parser = argparse.ArgumentParser(description=BANNER)
    where = parser.add_mutually_exclusive_group(required=True)
    where.add_argument("--port", help="serial port, e.g. COM3 (close IA-OS first)")
    where.add_argument("--sim", action="store_true", help="use the built-in simulator instead of hardware")
    parser.add_argument("--slave", type=int, default=1, help="Modbus address = axis number + 1 (default 1)")
    parser.add_argument(
        "--sim-profile", choices=sorted(PROFILES), default="slider",
        help="simulated unit with --sim (default slider)",
    )  # fmt: skip
    parser.add_argument(
        "--backup", type=Path, default=None,
        help="backup file (default backups/ec-backup-<port>-s<address>-<time>.json in the current folder)",
    )  # fmt: skip
    parser.add_argument(
        "--param", type=int, default=None,
        help="parameter number for the guarded write (default: 2 when it is writable and not dangerous, else the "
        "first parameter that is)",
    )  # fmt: skip
    parser.add_argument(
        "--force", action="store_true",
        help="also commit the unchanged value with force=True (one non-volatile write cycle)",
    )  # fmt: skip
    return parser


def open_axis(args: argparse.Namespace) -> Axis:
    """The axis on ``--port`` (baud rate found automatically), or a simulated unit with ``--sim``."""
    if args.sim:
        # pmsl=False: PIO in control, as after a power-up, so Modbus control is taken and handed back as on hardware.
        return Axis.open(SimulatedEC(slave=args.slave, profile=args.sim_profile, pmsl=False), slave=args.slave)
    return Axis.open(args.port, slave=args.slave)


def show(label: str, text: str) -> None:
    """One line of output: a short label, then the text."""
    print(f"{label:<10} {text}")


def backup_path(args: argparse.Namespace, slave: int) -> Path:
    """``--backup``, else ``backups/ec-backup-<port>-s<address>-<time>.json`` (the folder is created)."""
    if args.backup is not None:
        return args.backup
    label = "sim" if args.sim else "".join(c if c.isalnum() else "-" for c in args.port)
    path = Path("backups") / f"ec-backup-{label}-s{slave}-{dt.datetime.now():%Y%m%d-%H%M%S}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def label_of(prm: dict[str, Any]) -> str:
    """``P2`` on an ELECYLINDER, ``No.2`` on a CON-family controller."""
    return f"{'P' if prm['family'] == 'EC' else 'No.'}{prm['number']}"


def pick_parameter(params: list[dict[str, Any]], number: int | None) -> dict[str, Any]:
    """The parameter for the guarded write: ``number`` when given; else No. 2 when it is writable and not dangerous
    (the LS signal range on an ELECYLINDER), else the first parameter that is."""
    if number is not None:
        found = next((prm for prm in params if prm["number"] == number), None)
        if found is None:
            raise SystemExit(f"parameter {number} is not offered on this unit (see the list above)")
        return found
    usable = [prm for prm in params if prm["writable"] and not prm["dangerous"] and prm["value"] is not None]
    if not usable:
        raise SystemExit("this unit offers no writable parameter that is not dangerous")
    return next((prm for prm in usable if prm["number"] == 2), usable[0])


def print_write(call: str, result: ParameterWriteResult) -> None:
    """What one write_parameter() call did."""
    if result.skipped:
        show("write", f"{call}: skipped, {result.note}")
    elif result.committed and result.commit_confirmed:
        show("write", f"{call}: {result.before} -> {result.after}, read back and committed (the unit showed the "
                      f"commit); takes effect {result.takes_effect}")  # fmt: skip
    elif result.committed:
        show("write", f"{call}: {result.before} -> {result.after}, commit NOT confirmed (the unit never showed it "
                      "busy): check the value after a power cycle")  # fmt: skip
    else:
        show("write", f"{call}: {result.before} -> {result.after}, RAM only (not committed)")
    for warning in result.warnings:
        show("", f"note: {warning}")


def parameters_and_backup(ax: Axis, args: argparse.Namespace) -> None:
    """Back up, list and read the parameters, then the guarded write (and the forced one with ``--force``)."""
    found = ax.status()  # read-only: the state the unit was in before this program sent anything
    info = ax.connect(take_control=False)  # identity and ranges only: read-only
    show("unit", f"{info.model} ({ax.product_family}; actuator: {ax.kind}), firmware {info.firmware}, "
                 f"address {ax.slave}" + (f", {ax.baudrate} baud" if ax.baudrate else ""))  # fmt: skip

    path = backup_path(args, ax.slave)
    data = backup(ax, path)  # FC03 / FC11 only: nothing is written to the unit
    parameters = data.get("parameters") or {}
    items = sum(len(block.get("items") or []) for block in parameters.get("classes") or [])
    rows = (data.get("positions") or {}).get("rows") or []
    history = (data.get("alarms") or {}).get("history") or []
    show("backup", f"written to {data['meta'].get('path', path)} ({data['meta']['duration_s']} s, read-only)")
    show("", f"identity, directory, {items} parameter items, {len(rows)} position rows, {len(history)} alarm "
             "entries, maintenance counters")  # fmt: skip
    for error in data.get("errors") or []:
        show("", f"not read: {error}")

    params = ax.parameters()  # read-only
    if ax.is_ec:
        show("params", f"{len(params)} user parameters; {ax.parameter_model()['note']}")
    else:
        show("params", f"{len(params)} {ax.product_family} parameters (those whose item is in the unit's ID map)")
    for prm in params:
        flags = ("writable" if prm["writable"] else "read-only") + (", dangerous" if prm["dangerous"] else "")
        show("", f"{label_of(prm):<7} {prm['name']:<52} {prm['text'] or '-':<26} {flags}")

    prm = pick_parameter(params, args.param)
    number, value = prm["number"], prm["value"]
    one = ax.read_parameter(number)  # read-only, the same dict as in the list
    show("read", f"read_parameter({number}): {label_of(one)} {one['name']} = {one['text']} (item {one['item']} at "
                 f"{one['address']}, raw {one['raw']})")  # fmt: skip

    print_write(f"write_parameter({number}, {value!r})", ax.write_parameter(number, value))
    if args.force:
        print_write(f"write_parameter({number}, {value!r}, force=True)", ax.write_parameter(number, value, force=True))
    else:
        show("force", "not used: --force commits the unchanged value anyway (one non-volatile write cycle)")
    total, last_hour = ax.nonvolatile_writes, ax.nonvolatile_writes_last_hour  # counted per axis object
    show("counter", f"non-volatile writes by this axis object: {total} ({last_hour} in the last hour; one WARNING per "
                    f"hour above {NV_WARN_PER_HOUR}, never a refusal)")  # fmt: skip

    if not found.modbus_enabled and ax.status().modbus_enabled:
        ax.disable_modbus_control()  # the commit took Modbus control; PIO had it when this program started
        show("control", "handed back to PIO")


def report(exc: BaseException) -> None:
    """Print an error with the driver's notes."""
    sys.stdout.flush()  # keep the order when both streams go to one file
    print(f"error: {type(exc).__name__}: {exc}", file=sys.stderr)
    for note in getattr(exc, "__notes__", []):
        print(f"       {note}", file=sys.stderr)


def main(argv: Sequence[str] | None = None) -> int:
    """Entry point. Exit codes: 0 done, 2 error, 130 interrupted (Ctrl+C)."""
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.WARNING, format="  [driver %(levelname)s] %(message)s", stream=sys.stdout)
    if sys.platform == "win32":  # Ctrl+Break ends the program like Ctrl+C
        signal.signal(signal.SIGBREAK, signal.default_int_handler)
    print(BANNER)
    try:
        with open_axis(args) as ax:  # nothing here commands motion; the block always closes the port
            parameters_and_backup(ax, args)
    except KeyboardInterrupt:
        sys.stdout.flush()
        print("interrupted (Ctrl+C)", file=sys.stderr)
        return 130
    except (EcDriverError, OSError) as exc:
        report(exc)
        return 2
    print("done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
