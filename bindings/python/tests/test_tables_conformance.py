"""Conformance of the core's tables and decoders with those of the pure-Python edition, through a snapshot.

The driver takes the alarm names, levels, groups and remedies (``alarms``), the CON parameter table and its rules
(``con_params``) and the decoders of ``info`` from the shared C++ core (``iad_call_json``). The answers of the
pure-Python edition (removed after 1.0.x) are kept in ``tests/tables_snapshot.json``, so this check outlived it (the
only check of the core's alarm and CON tables against what the driver was verified with): every table in full, and the answers over the sweeps (every alarm code 0x000-0xFFF, every CON
parameter and a spread of raw values, inputs, families and unit limits, every bit table, the alarm-history and
maintenance records) as one digest per 64 answers. These tests compare the core's outputs with the snapshot: the same
values with the same Python types (250 and 250.0 differ), the same texts, the same refusals with the same messages.
A failed sweep names the 64 inputs that hold the difference and shows the core's answers to the first of them.

They need the native library (``IAI_ACTUATOR_DRIVER_NATIVE`` or ``core/build``), as the package does. A deliberate
change of a table or rule writes the snapshot again (from ``bindings/python``)::

    python tests/test_tables_conformance.py --write core     # the core's answers (a reviewed change of the core)

No hardware and no serial port.
"""

from __future__ import annotations

import dataclasses
import datetime as dt
import enum
import hashlib
import importlib
import json
import math
import random
import sys
import types
import unittest
from collections.abc import Callable, Iterator
from pathlib import Path
from typing import Any

if __name__ == "__main__":  # run as a script (--write): the package of this checkout
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from iai_actuator_driver import con_params, native  # noqa: E402
from iai_actuator_driver import registers as R  # noqa: E402
from iai_actuator_driver._shared import con_params as shared_con  # noqa: E402
from iai_actuator_driver._shared import info as shared_info  # noqa: E402
from iai_actuator_driver.values import IndexCategory  # noqa: E402

SNAPSHOT_FILE = Path(__file__).resolve().with_name("tables_snapshot.json")
CHUNK = 64  # answers per digest

try:
    _AVAILABLE = native.available()
except Exception:  # an import or load problem of the library: these tests are skipped, the others say why
    _AVAILABLE = False

if _AVAILABLE:
    from iai_actuator_driver._native import _abi_tables as T

    _AVAILABLE = T.has()

NEEDS_CORE = unittest.skipUnless(_AVAILABLE, "the native library (with iad_call_json) is not found: "
                                 "IAI_ACTUATOR_DRIVER_NATIVE")  # fmt: skip
CODES = range(0x000, 0x1000)
FAMILY_ARGS: list[tuple[int | None, str | None]] = [
    (0xEC, None), (None, None), (0xAE, None), (0xAC, None), (0xAA, None), (0xED, None), (None, "PCON"),
    (None, "acon"), (None, " DCON "), (None, "ACON/DCON"), (None, "scon"), (0xAE, "EC"), (0xEC, "PCON"), (None, ""),
]  # fmt: skip
CON_FAMILIES: list[Any] = [None, "PCON", "ACON", "DCON", "ACON/DCON", "SCON", "pcon", " scon ", "EC", "x", 0xAE,
                           0xAC, 0xAA, 0xEC, 0x00, True]  # fmt: skip
RAWS = [0, 1, 2, 3, 0x7F, 0x80, 0xFF, 0x100, 0x1FE, 0x2FD, 0xFFFF, 0x10000, 0x12345678, 0x7FFFFFFF, 0x80000000,
        0xFFFFFFFE, 0xFFFFFFFF, 999_999, 25_000, 230400, 21, 4, 5, 6, 7, 17, 18, 19, 20, 256]  # fmt: skip
ConUnitLimits = shared_con.ConUnitLimits
UNITS = [None, ConUnitLimits(max_speed=200.0, max_accel=0.7, max_push_speed=20.0),
         ConUnitLimits(lead_mm=12.0, encoder_pulses=800), ConUnitLimits(lead_mm=12.0),
         ConUnitLimits(lead_mm=6.0, encoder_pulses=8192, min_band_mm=0.05, max_speed=1234.56),
         ConUnitLimits(lead_mm=3.333333, encoder_pulses=1000, max_speed=12345.678)]  # fmt: skip
ALARM_TABLES = ("CONTROLLER_ALARM_NAMES", "CON_ALARM_NAMES", "MESSAGE_CODES", "CON_MESSAGE_CODES", "CON_SYSTEM_CODES",
                "ALARM_GROUPS", "GROUP_NAMES", "WARNING_CLEARING", "HOME_NEEDED", "LEVEL_TEXT", "FAULT_LEVELS",
                "RTC_EPOCH", "IAOS_ALARM_NAMES", "LEVEL_NONE", "LEVEL_MESSAGE", "LEVEL_OPERATION_CANCELLED",
                "LEVEL_COLD_START", "LEVEL_UNKNOWN", "__all__")  # fmt: skip
ALARM_PRIVATE = ("_SPECIFIC_REMEDY", "_GROUP_ADVICE", "_ABS_ERROR_DETAILS", "_ROW_FIELDS", "_COMMAND_FIELDS",
                 "_FAMILY_CLEARING", "_FAMILY_CLEARING_ANY", "_OPERATION_CANCELLED")  # fmt: skip
CON_TABLES = ("CON_PARAMETERS", "CON_PARAMETERS_BY_NO", "BAUD_CODES", "CON_BAUD_RATES", "CON_FAMILIES",
              "FAMILY_RANGES", "FAMILY_RANGE_DOC", "UNKNOWN_FAMILY_NOTE", "CATEGORIES", "CATEGORY_TEXT",
              "FACTORY_DEFAULTS", "HOME_PARAMETERS", "PUSH_SPEED_FLOOR_MM_S", "ENCODER_PULSES_ITEM", "__all__")
CON_PRIVATE = ("_FAMILY_CATEGORIES", "_FAMILY_DEFAULTS")
BIT_TABLE_NAMES = ("DSS1_BITS", "DSS2_BITS", "DSSE_BITS", "ZONS_BITS", "STAT_BITS", "SSSE_BITS", "SIPM_BITS",
                   "DRG1_BITS", "DRG2_BITS", "DRGE_BITS", "CTLF_BITS", "DOPM_BITS_3POS", "DOPM_BITS_2POS",
                   "DIPM_BITS_3POS", "DIPM_BITS_2POS", "DIPM_BITS_SINGLE")  # fmt: skip


# =========================================================================== the canonical form


def canon(value: Any) -> Any:
    """``value`` as JSON that keeps every Python type apart: numbers stay JSON numbers (``250`` and ``250.0`` differ in
    the text), a dict, tuple, set or record is a one-key object naming its type, a dict's items are sorted by key."""
    if value is None or isinstance(value, (bool, str, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else {"float": repr(value)}
    if isinstance(value, enum.Enum):
        return {"enum": f"{type(value).__name__}.{value.name}"}
    if isinstance(value, dict):
        items = [[canon(k), canon(v)] for k, v in value.items()]
        return {"dict": sorted(items, key=lambda kv: text(kv[0]))}
    if isinstance(value, list):
        return [canon(v) for v in value]
    if isinstance(value, tuple):
        return {"tuple": [canon(v) for v in value]}
    if isinstance(value, (set, frozenset)):
        return {type(value).__name__: sorted((canon(v) for v in value), key=text)}
    if isinstance(value, (dt.datetime, dt.date)):
        return {type(value).__name__: value.isoformat()}
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return {type(value).__name__: {f.name: canon(getattr(value, f.name)) for f in dataclasses.fields(value)}}
    return {"repr": repr(value)}


def text(value: Any) -> str:
    """The canonical text of a canonical value (what equality and the digests are taken over)."""
    return json.dumps(value, sort_keys=True, ensure_ascii=True, separators=(",", ":"))


def digest(answers: list[Any]) -> str:
    return hashlib.sha256(text([canon(a) for a in answers]).encode("ascii")).hexdigest()[:16]


def outcome(fn: Callable[..., Any], *args: Any, **kwargs: Any) -> tuple[str, Any]:
    """``("ok", result)`` or ``("ValueError", message)``: what a call gives, comparable across editions."""
    try:
        return "ok", fn(*args, **kwargs)
    except ValueError as exc:  # the refusals (both editions raise ValueError or a subclass)
        return "ValueError", str(exc)


def typed(value: Any) -> Any:
    """``value`` with the type of every number spelled out (``1 == 1.0`` would hide a wrong type)."""
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return value
    if isinstance(value, (int, float)):
        return (type(value).__name__, value)
    if isinstance(value, dict):
        return {k: typed(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return type(value).__name__, [typed(v) for v in value]
    if isinstance(value, (frozenset, set)):
        return type(value).__name__, sorted(value)
    if hasattr(value, "__dataclass_fields__"):
        return type(value).__name__, {f: typed(getattr(value, f)) for f in value.__dataclass_fields__}
    return value


# =========================================================================== the editions


class Edition:
    """One edition's ``alarms``, ``con_params`` and ``info`` modules, and its way of asking the alarm rules about many
    codes at once (the core: one ``iad_call_json`` per list, as the facades would take one call per code)."""

    def __init__(self, name: str, alarms: types.ModuleType, con: types.ModuleType, info: types.ModuleType) -> None:
        self.name, self.A, self.C, self.I = name, alarms, con, info

    def many(self, function: str, args: list[dict[str, Any]]) -> list[Any]:
        """``alarms.<function>`` for each keyword set of ``args``."""
        fn = getattr(self.A, function)
        return [fn(a["code"], **{k: v for k, v in a.items() if k != "code"}) for a in args]


class CoreEdition(Edition):
    def many(self, function: str, args: list[dict[str, Any]]) -> list[Any]:
        return T.call(f"alarms.{function}", args)


def core_edition() -> Edition:
    mods = [importlib.import_module(f"iai_actuator_driver.{m}") for m in ("alarms", "con_params", "info")]
    return CoreEdition("core", *mods)


# =========================================================================== the tables


def tables(ed: Edition) -> dict[str, Any]:
    """``{"<module>.<name>": canonical value}`` of every table the snapshot keeps in full. The private tables behind
    the rules are the pure-Python edition's names; the core keeps its own (``CoreTablesTest.test_the_cores_own_tables``)."""
    out: dict[str, Any] = {}
    for module, names in ((ed.A, ALARM_TABLES + ALARM_PRIVATE), (ed.C, CON_TABLES + CON_PRIVATE)):
        short = module.__name__.rpartition(".")[2]
        for name in names:
            if name.startswith("_") and not name.startswith("__") and not hasattr(module, name):
                continue
            value = getattr(module, name)
            out[f"{short}.{name}"] = canon(sorted(value) if name == "__all__" else value)
    out["info.BIT_TABLES"] = canon(ed.I.BIT_TABLES)
    out["info.BIT_TABLES order"] = canon(list(ed.I.BIT_TABLES))
    for name in BIT_TABLE_NAMES:
        out[f"info.{name}"] = canon(getattr(ed.I, name))
    out["info.PRODUCT_LINES"] = canon(ed.I.PRODUCT_LINES)
    out["info.ec_parameter_table()"] = canon(ed.I.ec_parameter_table())
    out["info.input_ranges()"] = canon(ed.I.input_ranges())
    out["info.input_ranges() order"] = canon(list(ed.I.input_ranges()))
    return out


# =========================================================================== the sweeps
# Each sweep yields (name, [(input label, answer)]); a group of sweeps is one test below.

Sweep = Iterator[tuple[str, list[tuple[str, Any]]]]


def alarm_names_and_levels(ed: Edition) -> Sweep:
    codes = [*CODES, 0x1000, 0x7FFF, 0xFFFF]
    for function in ("alarm_name", "alarm_group"):
        yield f"alarms.{function}", list(zip(map(hex, codes), ed.many(function, [{"code": c} for c in codes])))
    for absolute in (None, True, False):
        for system_code, family in FAMILY_ARGS:
            args = [{"code": c, "absolute_encoder": absolute, "system_code": system_code, "family": family or None}
                    for c in codes]  # fmt: skip
            yield (f"alarms.alarm_level absolute={absolute} system_code={system_code} family={family!r}",
                   list(zip(map(hex, codes), ed.many("alarm_level", args))))


def alarm_fault_rule(ed: Edition) -> Sweep:
    for alarm_bit in (False, True):
        for warning_bit in (False, True):
            for system_code, family in FAMILY_ARGS[:8]:
                args = [{"code": c, "alarm_bit": alarm_bit, "warning_bit": warning_bit, "system_code": system_code,
                         "family": family or None} for c in CODES]  # fmt: skip
                yield (f"alarms.alarm_is_fault alarm_bit={alarm_bit} warning_bit={warning_bit} "
                       f"system_code={system_code} family={family!r}",
                       list(zip(map(hex, CODES), ed.many("alarm_is_fault", args))))  # fmt: skip


def alarm_infos(ed: Edition) -> Sweep:
    named = set(ed.A.CONTROLLER_ALARM_NAMES) | set(R.ALARM_NAMES) | set(ed.A.CON_ALARM_NAMES)
    codes = sorted(set(range(0x000, 0x101)) | named | {0x1FF, 0x400, 0xFFE, 0xFFF, 0x1000})
    for absolute in (None, True, False):
        for system_code, family in [(0xEC, None), (None, None), (0xAE, None), (0xAC, None), (0xAA, None),
                                    (None, "PCON"), (None, "dcon"), (None, "SCON"), (0xAE, "EC")]:  # fmt: skip
            args = [{"code": c, "absolute_encoder": absolute, "system_code": system_code, "family": family}
                    for c in codes]  # fmt: skip
            yield (f"alarms.alarm_info absolute={absolute} system_code={system_code} family={family!r}",
                   list(zip(map(hex, codes), ed.many("alarm_info", args))))


def alarm_details(ed: Edition) -> Sweep:
    details = [None, 0, 1, 2, 3, 6, 0xB, 0x10, 0xFFFF]
    addresses = [None, 0, 0xFFFF, 0x0FFF, 0x1000, 0x1002, 0x100F, 0x1011, 0x2FFE, 0x3FFF, 0x4000, 0x0D0A,
                 *range(0x0F00, 0x0F10)]  # fmt: skip
    items = []
    for code in (0x0EE, 0x0A1, 0x0A2, 0x0A3, 0x090, 0x000):
        for detail in details:
            for address in addresses:
                items.append((f"{code:#x} {detail} {address}", (ed.A.describe_alarm_detail(code, detail, address),
                                                                ed.A.alarm_info(code, detail=detail, address=address))))
    yield "alarms.describe_alarm_detail and alarm_info(detail, address)", items


def alarm_history_and_times(ed: Edition) -> Sweep:
    A = ed.A
    rng = random.Random(3)
    pool = [0, 0x0FFF, 0x0EE, 0x0A2, 0x048, 0x02C, 0x123, 0x0D9]
    items = []
    for n in range(60):
        words: list[int] = []
        for _ in range(rng.randrange(0, 9)):
            stamp = rng.choice([0xFFFFFFFF, 0, 1, rng.randrange(0, 1 << 32)])
            words += [rng.randrange(0, 1 << 16), rng.choice([0xFFFF, 0x1023, 0x0F02, 0]), 0, rng.choice(pool),
                      stamp >> 16, stamp & 0xFFFF]  # fmt: skip
        words += [0] * rng.randrange(0, 6)  # a partial slot is not decoded
        for include_empty in (False, True):
            items.append((f"{n} {include_empty}", A.alarm_history_slots(words, include_empty=include_empty)))
    yield "alarms.alarm_history_slots", items
    items = []
    for raw in (None, 0, 1, 999, 1000, 59_999, 3_599_999, 86_399_999, 86_400_000, 1_234_567, 0xFFFFFFFE):
        for rtc in (None, False, True):
            items.append((f"alarm_time {raw} {rtc}", A.alarm_time(raw, rtc_in_use=rtc)))
        if raw is not None:
            items.append((f"rtc_datetime {raw}", A.rtc_datetime(raw)))
    for system_code in (None, 0, 0xEC, 0xED, 0xAE, 0xAC, 0xAA, 0x11):
        for family in (None, "", "pcon", " ACON ", "DCON", "acon/dcon", "SCON", "EC", "IAI controller"):
            items.append((f"alarm_family {system_code} {family!r}", A.alarm_family(system_code, family)))
    for support in (None, {}, {"SUPT": 0}, {"SUPT": 1 << 13}, {"SUP2": 1 << 18}, {"SUP2": 0},
                    {"SUPT": 0, "SUP2": 1 << 18}, {"SUPT": 0x60002BFF, "SUP2": 0x838C009A}, {"SUP3": 1}):  # fmt: skip
        items.append((f"absolute_encoder_from_support {support}", A.absolute_encoder_from_support(support)))
    yield "alarms.alarm_time, rtc_datetime, alarm_family, absolute_encoder_from_support", items
    items = []
    for code in (0, 0x02C, 0x048, 0x069, 0x090, 0x0CE, 0x0D2, 0x0EE, 0x0F1, 0x123, 0xFFF):
        for kwargs in ({}, {"family": "PCON"}, {"system_code": 0xAA}, {"absolute_encoder": False}):
            items.append((f"{code:#x} {kwargs}", (A.alarm_info(code, **kwargs), A.alarm_level(code, **kwargs))))
        items.append((f"{code:#x}", (A.alarm_name(code), A.alarm_group(code), A.alarm_is_fault(code, warning_bit=True))))
    yield "alarms: the module functions", items


def inputs_for(p: Any) -> list[Any]:
    """Values to parse / check for ``p``: its bounds and around them, labels, numeric strings and odd types."""
    values: list[Any] = [0, 1, 2, 3, 7, -1, 1.5, -1.0, 0.005, 0.015, 0.125, 10.0, 250, 251, 300, 9600, 57600, 38400,
                         1e10, float("inf"), float("nan"), "1", "1.5", "abc", "Forward", "1:Forward", "for", "",
                         " 2 ", "-3", "1.0", "0x10", True, None, [1], "Enable", "en", "Disable"]  # fmt: skip
    for bound in (p.lo, p.hi):
        if bound is not None:
            values += [bound, bound + 0.004, bound - 0.004, bound + 1, bound - 1, str(bound)]
    values += list(p.choices.values()) + [f"{k}:{v}" for k, v in p.choices.items()]
    return values


def con_families(ed: Edition) -> Sweep:
    C = ed.C
    for family in CON_FAMILIES:
        items: list[tuple[str, Any]] = [("con_family", C.con_family(family)), ("con_parameters", C.con_parameters(family)),
                                        ("parameter_table", C.parameter_table(family))]  # fmt: skip
        for no in range(0, 260):
            items.append((f"category/factory_default {no}", (C.category(no, family), C.factory_default(no, family))))
        for p in C.CON_PARAMETERS:
            items.append((f"for_family {p.no}", C.for_family(p, family)))
        yield f"con_params by family {family!r}", items


def con_decode_encode(ed: Edition) -> Sweep:
    C = ed.C
    rng = random.Random(7)
    raws = RAWS + [rng.randrange(0, 1 << 32) for _ in range(40)]
    items = []
    for p in C.CON_PARAMETERS:
        for raw in raws:
            value = C.decode(p, raw)
            encoded = outcome(C.encode, p, raw, value) if value is not None and p.writable else None
            items.append((f"No.{p.no} raw {raw:#x}", (value, C.text(p, value), encoded)))
        for value in (None, 1.0, 2.5, 0, 1, 5):
            items.append((f"No.{p.no} text {value!r}", outcome(C.text, p, value)))
    yield "con_params.decode, text, encode", items


def con_parse_and_check(ed: Edition) -> Sweep:
    C = ed.C
    items = []
    for p in C.CON_PARAMETERS:
        for value in inputs_for(p):
            for family in (None, "PCON", "ACON", "SCON", 0xAA, "EC"):
                items.append((f"No.{p.no} {value!r} {family!r}", outcome(C.parse, p, value, family=family)))
    yield "con_params.parse", items
    items = []
    for p in C.CON_PARAMETERS:
        for value in inputs_for(p):
            for family in (None, "PCON", "ACON", "SCON"):
                for k, unit in enumerate(UNITS):
                    items.append((f"No.{p.no} {value!r} {family!r} unit {k}",
                                  outcome(C.check_value, p, value, family=family, unit=unit)))  # fmt: skip
    yield "con_params.check_value", items


def con_audit_band_limits(ed: Edition) -> Sweep:
    C = ed.C
    rng = random.Random(11)
    items = []
    for n in range(12):
        values = {p.no: C.decode(p, rng.choice(RAWS)) for p in C.CON_PARAMETERS}
        values[999] = 5  # not a CON parameter: left out
        for family in (None, "PCON", "ACON/DCON", "SCON"):
            for k, unit in enumerate(UNITS):
                items.append((f"{n} {family!r} unit {k}", C.audit(values, family=family, unit=unit)))
    yield "con_params.audit", items
    items = []
    for family in (None, "PCON", "ACON", "SCON", 0xAE, "EC"):
        for lead in (None, 0, 0.0, 12.0, 6.0, 2.5, 20.0, 3.333333, 1.0):
            for pulses in (None, 0, 800, 1000, 4095, 4096, 8192):
                for mini in (None, 0, 0.0, 0.03, 0.2, 0.035):
                    items.append((f"{family!r} {lead} {pulses} {mini}",
                                  C.min_positioning_band(family, lead, pulses, mini_mm=mini)))  # fmt: skip
    yield "con_params.min_positioning_band", items
    rng = random.Random(5)
    ClassItem, ClassBlock = shared_info.ClassItem, shared_info.ClassBlock

    def block(ci: int, ids: list[int]) -> Any:
        found = []
        for k, item_id in enumerate(ids):
            raw = rng.choice([0, 0xFFFFFFFF, 0x80000001, rng.randrange(1, 100_000), rng.randrange(0, 1 << 32)])
            found.append(ClassItem(ci, item_id, 0, k, 0x0800 + 2 * k, 2, raw))
        return ClassBlock(ci, 0x0800, 2 * len(ids), 0, 1, 0x0700, tuple(found))

    items = []
    for n in range(40):
        classes = {
            R.CATEGORY_LIMIT_PARAMS: block(R.CATEGORY_LIMIT_PARAMS, list(range(1, 12))),
            R.CATEGORY_FACTORY_PARAMS: block(R.CATEGORY_FACTORY_PARAMS, list(range(10, 14))),
            R.CATEGORY_CONTROLLER_MONITOR: block(R.CATEGORY_CONTROLLER_MONITOR, [R.CONTROLLER_ITEM_MINI, 1]),
        }
        if n % 4 == 3:
            del classes[R.CATEGORY_FACTORY_PARAMS]
        for support in (None, {}, {"SUP2": R.SUP2_CINP}, {"SUP2": 0}, {"SUP2": 0xFFFFFFFF}):
            items.append((f"{n} {support}", C.con_unit_limits(classes, support)))
    yield "con_params.con_unit_limits", items


def info_bits_and_modes(ed: Edition) -> Sweep:
    I = ed.I  # noqa: E741
    rng = random.Random(1)
    items = []
    for name in I.BIT_TABLES:
        for width in (16, 32):
            words = [0, (1 << width) - 1, 1, 1 << (width - 1), *(rng.randrange(0, 1 << width) for _ in range(30))]
            for word in words:
                items.append((f"{name} {width} {word:#x}", I.decode_bits(word, name, width=width)))
    BitDef = shared_info.BitDef
    custom = (BitDef(3, "X", "meaning", True, "e", True), BitDef(0, "Y"))
    items.append(("custom table", I.decode_bits(0x0F, custom)))
    for word, width in ((-1, 16), (1 << 16, 16), (1 << 8, 8)):
        items.append((f"DSS1 {word} {width}", outcome(I.decode_bits, word, "DSS1", width=width)))
    yield "info.decode_bits", items
    items = []
    for code in (None, 0, 1, 2, 3, 4, 5, 99):
        for model in (None, "", "EC-S6-MF", "EC-GRBP8M-20", "ec-r6mf", "MFX", "EC-S6SAH-50-2-MF-X"):
            mode = I.position_mode(code, model)
            items.append((f"{code} {model!r}", (mode, I.io_bit_tables(mode))))
    for layout in ("3pos", "2pos", "x"):
        items.append((f"io_bit_tables {layout}", I.io_bit_tables(layout)))
    for ssse in (None, 0, 1 << 8, 0xFFFF, 0xFEFF):
        items.append((f"rtc_in_use {ssse}", I.rtc_in_use(ssse)))
    yield "info.position_mode, io_bit_tables, rtc_in_use", items


def maintenance_block(rng: random.Random, timn: int | None) -> Any:
    items = []
    address = 0x8400
    for k, item_id in enumerate([*range(1, 31), None]):
        size = 1 if item_id in (9, None) and rng.random() < 0.5 else 2
        if timn is not None and item_id == 18:
            address = timn
        raw = rng.choice([0, 0xFFFFFFFF, 3034, rng.randrange(0, 1 << (16 * size))])
        items.append(shared_info.ClassItem(0x0C, item_id, 0, k, address, size, raw))
        address += size
    return shared_info.ClassBlock(0x0C, 0x8400, address - 0x8400, 0, 1, 0x0710, tuple(items))


def info_records(ed: Edition) -> Sweep:
    I = ed.I  # noqa: E741
    rng = random.Random(9)
    items = []
    for n in range(30):
        timn = rng.choice([None, 0x841E, 0x8420, 0x8422])
        block = maintenance_block(rng, timn)
        for rtc in (None, False, True):
            for family in (None, "PCON", "SCON", "ACON", "DCON", "EC", 0xAE, 0xAA):
                items.append((f"{n} {rtc} {family!r}", I.decode_maintenance(block, rtc=rtc, family=family)))
    yield "info.decode_maintenance", items
    rng = random.Random(4)
    pool = [0, 0x0FFF, 0x0EE, 0x0A2, 0x0A3, 0x048, 0x02C, 0x123, 0x0D9, 0x0CE]
    items = []
    for n in range(40):
        rec = IndexCategory(5, 0, 6 if n % 7 != 6 else 4, 32, 0x0500, 0, 0, 1)
        words: list[int] = []
        for _ in range(rng.randrange(0, 10)):
            stamp = rng.choice([0xFFFFFFFF, 0, rng.randrange(0, 1 << 32)])
            words += [rng.choice([0, 1, 2, 6, 0xB, rng.randrange(0, 1 << 16)]),
                      rng.choice([0xFFFF, 0x1023, 0x0F02, 0x0612, 0]), 0, rng.choice(pool), stamp >> 16,
                      stamp & 0xFFFF]  # fmt: skip
        words += [0] * (6 * rng.randrange(0, 3))
        current = rng.choice([None, 0, 0x0EE, 0x02C, 0x123, 0x048, 0x0CE])
        dss1 = rng.choice([None, 0, R.DSS1_ALMH, R.DSS1_ALML, R.DSS1_ALMH | R.DSS1_ALML])
        c3 = rng.choice([None, 0, 0x0EE])
        for absolute in (None, True, False):
            for rtc in (None, True):
                for system_code in (None, 0xAE, 0xAA, 0xEC):
                    items.append((f"{n} {absolute} {rtc} {system_code}",
                                  I.decode_alarm_history(rec, words, current, dss1, c3, absolute, rtc_in_use=rtc,
                                                         system_code=system_code)))  # fmt: skip
    yield "info.decode_alarm_history", items


def info_ranges(ed: Edition) -> Sweep:
    I = ed.I  # noqa: E741
    items = []
    for line in (*I.PRODUCT_LINES, "RCON-EC", "ec", 5):
        for family in (None, "PCON", "ACON", " scon ", "EC", "x", 0xAE, 0xAC, 0xEC, True):
            items.append((f"input_ranges {line!r} {family!r}", outcome(I.input_ranges, line, family=family)))
    for family in (None, "", "EC", "ec", "PCON", "IAI controller", 0xEC):
        for gateway in (None, "", "rcon", "RCON-GW", "robonet-gateway", "ROBONET", "other"):
            items.append((f"product_line {family!r} {gateway!r}", I.product_line(family, gateway)))
    yield "info.input_ranges, product_line", items


GROUPS: dict[str, Callable[[Edition], Sweep]] = {
    "alarm_names_and_levels": alarm_names_and_levels,
    "alarm_fault_rule": alarm_fault_rule,
    "alarm_infos": alarm_infos,
    "alarm_details": alarm_details,
    "alarm_history_and_times": alarm_history_and_times,
    "con_families": con_families,
    "con_decode_encode": con_decode_encode,
    "con_parse_and_check": con_parse_and_check,
    "con_audit_band_limits": con_audit_band_limits,
    "info_bits_and_modes": info_bits_and_modes,
    "info_records": info_records,
    "info_ranges": info_ranges,
}


def chunk_digests(items: list[tuple[str, Any]]) -> list[str]:
    return [digest([a for _, a in items[i:i + CHUNK]]) for i in range(0, len(items), CHUNK)]


def snapshot_of(ed: Edition) -> dict[str, Any]:
    sweeps: dict[str, Any] = {}
    for group in GROUPS.values():
        for name, items in group(ed):
            sweeps[name] = {"n": len(items), "chunks": chunk_digests(items)}
    return {"tables": tables(ed), "sweeps": sweeps}


def write_snapshot(ed: Edition, path: Path = SNAPSHOT_FILE) -> None:
    """The snapshot as JSON with one line per table and per sweep (a change shows as the lines it touches)."""
    data = snapshot_of(ed)
    about = f"written from the {ed.name} edition by tests/test_tables_conformance.py --write {ed.name}"
    lines = ["{", f' "about": {json.dumps(about)},']
    for part in ("sweeps", "tables"):
        entries = [f"  {json.dumps(k)}: {json.dumps(v, sort_keys=True, ensure_ascii=True, separators=(',', ':'))}"
                   for k, v in sorted(data[part].items())]  # fmt: skip
        lines += [f" {json.dumps(part)}: {{", ",\n".join(entries), " }" + ("," if part == "sweeps" else "")]
    path.write_text("\n".join(lines) + "\n}\n", encoding="utf-8", newline="\n")


def read_snapshot() -> dict[str, Any]:
    return json.loads(SNAPSHOT_FILE.read_text(encoding="utf-8"))


# =========================================================================== the tests


class _Compare(unittest.TestCase):
    """Compare an edition's tables and sweeps with the snapshot."""

    snapshot: dict[str, Any]

    @classmethod
    def setUpClass(cls) -> None:
        if not SNAPSHOT_FILE.is_file():  # a copy of the tests without their data (an sdist)
            raise unittest.SkipTest(f"{SNAPSHOT_FILE.name} is not next to the tests")
        cls.snapshot = read_snapshot()

    def edition(self) -> Edition:
        raise NotImplementedError

    def check_tables(self, prefix: str) -> None:
        ours = tables(self.edition())
        for name, want in self.snapshot["tables"].items():
            private = name.rpartition(".")[2][:1] == "_" and not name.endswith(".__all__")
            if name.startswith(prefix) and not (private and name not in ours):
                with self.subTest(table=name):
                    self.assertIn(name, ours)
                    self.assertEqual(text(ours[name]), text(want), f"{name} differs from the snapshot")

    def check_group(self, group: str) -> None:
        for name, items in GROUPS[group](self.edition()):
            with self.subTest(sweep=name):
                want = self.snapshot["sweeps"].get(name)
                self.assertIsNotNone(want, f"{name}: not in the snapshot (write it again)")
                self.assertEqual(len(items), want["n"], f"{name}: {len(items)} inputs, the snapshot has {want['n']}")
                for k, (got, expected) in enumerate(zip(chunk_digests(items), want["chunks"])):
                    if got != expected:
                        part = items[k * CHUNK:(k + 1) * CHUNK]
                        shown = "\n".join(f"  {label}: {text(canon(a))[:300]}" for label, a in part[:8])
                        self.fail(f"{name}: an answer of inputs {k * CHUNK}..{k * CHUNK + len(part) - 1} "
                                  f"({part[0][0]} .. {part[-1][0]}) differs from the snapshot; the {self.edition().name}"
                                  f"'s first answers there:\n{shown}")


class _Groups:
    """One test per table module and per group of sweeps."""

    def test_alarm_tables(self: Any) -> None:
        self.check_tables("alarms.")

    def test_con_tables(self: Any) -> None:
        self.check_tables("con_params.")

    def test_info_tables(self: Any) -> None:
        self.check_tables("info.")

    def test_alarm_names_and_levels(self: Any) -> None:
        self.check_group("alarm_names_and_levels")

    def test_alarm_fault_rule(self: Any) -> None:
        self.check_group("alarm_fault_rule")

    def test_alarm_infos(self: Any) -> None:
        self.check_group("alarm_infos")

    def test_alarm_details(self: Any) -> None:
        self.check_group("alarm_details")

    def test_alarm_history_and_times(self: Any) -> None:
        self.check_group("alarm_history_and_times")

    def test_con_families(self: Any) -> None:
        self.check_group("con_families")

    def test_con_decode_encode(self: Any) -> None:
        self.check_group("con_decode_encode")

    def test_con_parse_and_check(self: Any) -> None:
        self.check_group("con_parse_and_check")

    def test_con_audit_band_limits(self: Any) -> None:
        self.check_group("con_audit_band_limits")

    def test_info_bits_and_modes(self: Any) -> None:
        self.check_group("info_bits_and_modes")

    def test_info_records(self: Any) -> None:
        self.check_group("info_records")

    def test_info_ranges(self: Any) -> None:
        self.check_group("info_ranges")


@NEEDS_CORE
class CoreTablesTest(_Groups, _Compare):
    """The core's tables and rules (``iad_call_json``, through the native modules) against the snapshot."""

    def edition(self) -> Edition:
        return core_edition()

    def test_the_cores_own_tables(self) -> None:
        """The tables the core keeps behind its rules, against the snapshot's private tables."""
        want = self.snapshot["tables"]
        core = T.call("alarms.tables")
        self.assertEqual(dict(core["curated_names"]), R.ALARM_NAMES)
        clearing: dict[int, dict[str, str]] = {}
        for code, family, label in core["family_clearing"]:
            clearing.setdefault(code, {})[family] = label
        mine = {"_SPECIFIC_REMEDY": dict(core["specific_remedies"]), "_GROUP_ADVICE": dict(core["group_advice"]),
                "_ABS_ERROR_DETAILS": dict(core["abs_error_details"]), "_ROW_FIELDS": dict(core["row_fields"]),
                "_COMMAND_FIELDS": dict(core["command_fields"]), "_FAMILY_CLEARING": clearing,
                "_FAMILY_CLEARING_ANY": core["family_clearing_any"],
                "_OPERATION_CANCELLED": frozenset(core["operation_cancelled_codes"]) | {0x0EE}}  # fmt: skip
        snapshot_cancelled = {v for v in want["alarms._OPERATION_CANCELLED"]["frozenset"]} | {0x0EE}
        for name, value in mine.items():
            with self.subTest(table=name):
                if name == "_OPERATION_CANCELLED":
                    self.assertEqual(set(value), snapshot_cancelled)
                else:
                    self.assertEqual(text(canon(value)), text(want[f"alarms.{name}"]))
        self.assertEqual(core["power_up_code"], R.ALARM_POWER_UP)
        self.assertEqual(core["history_stride"], R.ALARM_HISTORY_STRIDE)
        con = T.call("con_params.tables")
        self.assertEqual(text(canon({f: dict(rows) for f, rows in con["family_categories"].items()})),
                         text(want["con_params._FAMILY_CATEGORIES"]))  # fmt: skip
        self.assertEqual(text(canon({f: dict(rows) for f, rows in con["family_defaults"].items()})),
                         text(want["con_params._FAMILY_DEFAULTS"]))  # fmt: skip

    def test_shared_value_types_and_identities(self) -> None:
        ed = self.edition()
        for cls in ("ConParam", "ConUnitLimits", "MinBand", "ConCheck"):
            self.assertIs(getattr(ed.C, cls), getattr(shared_con, cls))  # the shared value types
        for cls in ("BitDef", "ClassItem", "ClassBlock"):
            self.assertIs(getattr(ed.I, cls), getattr(shared_info, cls))
        self.assertIs(ed.I.io_bit_tables("3pos")["DIPM"], ed.I.DIPM_BITS_3POS)
        with self.assertRaises(KeyError):
            ed.I.decode_bits(0, "NOPE")
        self.assertIsInstance(ed.A.rtc_datetime(1000), dt.datetime)


@NEEDS_CORE
class StatusDecodeTest(unittest.TestCase):
    """``values.Status.from_registers`` stays Python (raw words in the GUI and the tests): it decodes
    as the core does, over random blocks of every length read."""

    def test_random_words(self) -> None:
        from iai_actuator_driver.values import Status

        rng = random.Random(2)
        for n in range(1500):
            words = [rng.randrange(0, 1 << 16) for _ in range(rng.choice([8, 9, 13, 14, 19, 20, 21, 22, 24]))]
            if n % 5 == 0:
                words[2] = rng.choice([0, 0x0FFF, 0x0EE, 0x02C, 0x048, 0x090, 0x123])
            with self.subTest(words=words):
                self.assertEqual(typed(T.call("status.decode", {"words": words})),
                                 typed(Status.from_registers(words).to_dict()))  # fmt: skip
        for words in ([], [0] * 7):
            self.assertEqual(outcome(T.call, "status.decode", {"words": words}),
                             outcome(Status.from_registers, words))  # fmt: skip


class SharedNamesTest(unittest.TestCase):
    """What the table in use says."""

    def test_float_repr_of_the_bounds(self) -> None:
        # the core writes the bounds the table spells as floats (0.0, 1.0, 100.0) the same way in its messages
        floats = sorted(p.no for p in con_params.CON_PARAMETERS
                        for b in (p.lo, p.hi) if isinstance(b, float) and b == math.floor(b))  # fmt: skip
        self.assertEqual(floats, [22, 48, 49, 55, 55, 88])


if __name__ == "__main__":
    if sys.argv[1:] == ["--write", "core"]:
        if not _AVAILABLE:
            sys.exit("the native library (with iad_call_json) is not found: IAI_ACTUATOR_DRIVER_NATIVE")
        write_snapshot(core_edition())
        print(f"wrote {SNAPSHOT_FILE}")
    else:
        unittest.main()
