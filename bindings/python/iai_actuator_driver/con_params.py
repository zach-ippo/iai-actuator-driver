"""Numbered parameters of IAI CON-family controllers (PCON, ACON, DCON, SCON ...), as IAI's PC tool numbers them.

Each entry names the parameter number, its symbol, where it lives (class.item, found through the unit's own ID map,
so the address may differ per model), its encoding and range, and whether a write is dangerous (home, motor / encoder,
lead / axis type, servo gains, PIO / fieldbus / baud configuration): :meth:`iai_actuator_driver.ec.Elecylinder.write_parameter`
writes those with a warning. A parameter is offered only when the unit's ID map has its item.
Read on a PCON-CB + RCP6 (VERIFIED: values and item locations); the encodings of items that were not changed on
hardware are UNVERIFIED.

Encodings (``kind``): ``scaled`` raw x ``scale`` (``signed`` s32); ``raw`` the integer; ``pct255`` raw 0-255 =
0-100 %; ``bits`` the value is the bit field ``bits`` (value bit j = item bit ``bits[j]``, read-modify-write);
``pair3`` two bits as 0 / 1 / 2 (raw 3 -> 2); ``baud`` an IAI baud code shown as bit/s; ``parity`` the home pattern
number (odd = 1 Forward, even = 0 Reverse; a change writes the pattern -1 / +1); ``ipv4`` an address, first octet in
the high byte; ``flag`` read-only here (non-zero = 1).

Ranges per family (DOC). The table's ``lo`` / ``hi`` are the PCON-CB ranges (ME0342-4H p.6-2 to 6-6, details
p.6-7 to 6-57). :func:`for_family` gives the ACON / DCON-CB ranges (ME0343-3B p.6-2 to 6-6) and the SCON-CB ranges
(ME0340-9F p.6-2 to 6-9); any other family (and a unit whose family is not known) uses the PCON-CB range, with a
note that says so. No.16 (SIO baud) takes only the eight documented rates (ME0342-4H p.6-14; ME0343-3B p.6-14;
ME0340-9F p.6-16). :func:`check_value` adds what needs the unit itself (:class:`ConUnitLimits`,
:func:`con_unit_limits`): No.8 / 26 / 35 / 47 capped by the actuator's maximum speed MAXV (7.4), No.9 by MAXA (7.5),
No.34 by the maximum push speed MXPH (7.8), and No.10 not below the minimum positioning band
(:func:`min_positioning_band`): a smaller No.10 makes the next software reset raise cold-start alarm 0x0A1
(ME0342-4H p.6-12, p.8-22). :data:`CATEGORIES` holds IAI's parameter categories (A check, B per application, C leave
as shipped, D actuator-specific, E never change; ME0342-4H p.6-2): every D and E parameter is ``dangerous``.
:func:`parameter_table` is the per-family table a GUI can show.

The table, the per-family ranges, categories and factory defaults, and every rule (decode / encode / parse / text,
the unit's own maxima, the minimum positioning band, check_value / audit) are the core's (``core/src/
con_parameters.cpp``), reached through ``iad_call_json`` (:mod:`._abi_tables`); the tables below are read from the core
once, at import. The value types (:class:`ConParam`, :class:`ConUnitLimits`, :class:`MinBand`, :class:`ConCheck`) are
the shared classes (``_shared/con_params.py``). Kinds, evidence and manual pages: above;
``tests/test_tables_conformance.py`` compares the core's answers with the tables of 1.0 (``tests/tables_snapshot.json``).
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace
from typing import Any, Final

from ._shared.con_params import ConCheck, ConParam, ConUnitLimits, MinBand
from ._native import _abi_tables as _t

__all__ = [
    "ConParam",
    "CON_PARAMETERS",
    "CON_PARAMETERS_BY_NO",
    "BAUD_CODES",
    "CON_BAUD_RATES",
    "CON_FAMILIES",
    "FAMILY_RANGES",
    "FAMILY_RANGE_DOC",
    "CATEGORIES",
    "CATEGORY_TEXT",
    "FACTORY_DEFAULTS",
    "HOME_PARAMETERS",
    "UNKNOWN_FAMILY_NOTE",
    "ConUnitLimits",
    "ConCheck",
    "MinBand",
    "con_family",
    "for_family",
    "con_parameters",
    "category",
    "factory_default",
    "con_unit_limits",
    "min_positioning_band",
    "check_value",
    "audit",
    "parameter_table",
    "decode",
    "encode",
    "parse",
    "text",
]


def _param(row: Mapping[str, Any]) -> ConParam:
    """A :class:`ConParam` from the core's JSON."""
    return ConParam(
        row["no"], row["name"], row["symbol"], row["class_id"], row["item_id"], row["kind"], entry=row["entry"],
        scale=row["scale"], signed=row["signed"], bits=tuple(row["bits"]), unit=row["unit"], lo=row["lo"],
        hi=row["hi"], choices={int(k): v for k, v in row["choices"]}, dangerous=row["dangerous"],
        writable=row["writable"], note=row["note"],
    )  # fmt: skip


def _json(p: ConParam) -> dict[str, Any]:
    """A :class:`ConParam` as the core takes it (the bounds keep their int / float type)."""
    return {
        "no": p.no, "name": p.name, "symbol": p.symbol, "class_id": p.class_id, "item_id": p.item_id, "kind": p.kind,
        "entry": p.entry, "scale": p.scale, "signed": p.signed, "bits": list(p.bits), "unit": p.unit, "lo": p.lo,
        "hi": p.hi, "choices": [[k, v] for k, v in p.choices.items()], "dangerous": p.dangerous,
        "writable": p.writable, "note": p.note,
    }  # fmt: skip


def _family(family: str | int | None) -> Any:
    """A family argument as the core takes it: a name, an FC11 system code, or anything else (no CON family)."""
    if family is None or isinstance(family, (bool, int, str)):
        return family
    return str(family)


def _unit(unit: ConUnitLimits | None) -> dict[str, Any] | None:
    return None if unit is None else unit.to_dict()


# The tables are read from the core at the first use of one of them (native.py imports this module while the library
# is being loaded, so nothing here calls the core at import).
_TABLE_NAMES: Final = frozenset({
    "BAUD_CODES", "CON_BAUD_RATES", "CON_PARAMETERS", "CON_PARAMETERS_BY_NO", "CON_FAMILIES", "FAMILY_RANGES",
    "FAMILY_RANGE_DOC", "UNKNOWN_FAMILY_NOTE", "CATEGORIES", "CATEGORY_TEXT", "FACTORY_DEFAULTS", "HOME_PARAMETERS",
})  # fmt: skip


def _tables() -> dict[str, Any]:
    """The CON tables from the core: the parameters (PCON-CB ranges), the baud codes and the eight SIO rates No.16
    takes, the families with a range table and their ranges / manual pages, IAI's categories, the factory defaults
    that do not depend on the actuator, the home parameters (No.5 ORG, No.22 OFST)."""
    t = _t.call("con_params.tables")
    params = tuple(_param(row) for row in t["parameters"])
    return {
        "BAUD_CODES": dict(t["baud_codes"]), "CON_BAUD_RATES": tuple(t["con_baud_rates"]),
        "CON_PARAMETERS": params, "CON_PARAMETERS_BY_NO": {p.no: p for p in params},
        "CON_FAMILIES": tuple(t["families"]),
        "FAMILY_RANGES": {f: {no: (lo, hi) for no, lo, hi in rows} for f, rows in t["family_ranges"].items()},
        "FAMILY_RANGE_DOC": dict(t["family_range_doc"]), "UNKNOWN_FAMILY_NOTE": t["unknown_family_note"],
        "CATEGORIES": dict(t["categories"]), "CATEGORY_TEXT": dict(t["category_text"]),
        "FACTORY_DEFAULTS": dict(t["factory_defaults"]), "HOME_PARAMETERS": frozenset(t["home_parameters"]),
    }  # fmt: skip


def __getattr__(name: str) -> Any:
    if name in _TABLE_NAMES:
        globals().update(_tables())
        return globals()[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__() -> list[str]:
    return sorted(set(globals()) | _TABLE_NAMES)


PUSH_SPEED_FLOOR_MM_S: Final = 5.0  # No.34: "the limit should be 5 mm/s" (ME0342-4H p.6-23): a warning (core rule)
ENCODER_PULSES_ITEM: Final = 12  # class 9 item 12 EPLS: encoder pulses / rev (read by the core's con_unit_limits)


def con_family(family: str | int | None) -> str | None:
    """The range-table family of a unit: ``"PCON"``, ``"ACON/DCON"`` or ``"SCON"`` from a family name (``"PCON"``,
    ``"ACON"``, ``"DCON"``, ``"ACON/DCON"``, ``"SCON"``; case-insensitive) or an FC11 system code; ``None`` for
    anything else (an EC, a generic IAI controller, unknown)."""
    return _t.call("con_params.con_family", {"family": _family(family)})  # type: ignore[no-any-return]


def for_family(p: ConParam, family: str | int | None) -> ConParam:
    """``p`` with the range of ``family`` (:func:`con_family`; :data:`FAMILY_RANGES`). An unknown family keeps the
    PCON-CB range (:data:`UNKNOWN_FAMILY_NOTE`); a D / E category of that family makes it dangerous."""
    out = replace(_param(_t.call("con_params.for_family", {"p": _json(p), "family": _family(family)})), note=p.note)
    return p if out == p else out


def con_parameters(family: str | int | None = None) -> tuple[ConParam, ...]:
    """Every CON parameter with the ranges of ``family`` (:func:`for_family`)."""
    return tuple(_param(row) for row in _t.call("con_params.con_parameters", {"family": _family(family)}))


def category(no: int, family: str | int | None = None) -> str:
    """IAI's category letter of parameter ``no`` (A-E, :data:`CATEGORY_TEXT`), ``""`` when not listed."""
    return _t.call("con_params.category", {"no": no, "family": _family(family)})  # type: ignore[no-any-return]


def factory_default(no: int, family: str | int | None = None) -> int | float | None:
    """The documented factory default of parameter ``no`` when it does not depend on the actuator, else ``None``."""
    return _t.call(  # type: ignore[no-any-return]
        "con_params.factory_default", {"no": no, "family": _family(family)}
    )


def _block(block: Any) -> dict[str, Any]:
    """A class block (:class:`iai_actuator_driver.info.ClassBlock`) as the core takes it."""
    return {
        "class_id": block.class_id, "address": block.address, "words": block.words, "member_size": block.member_size,
        "entries": block.entries, "id_map_address": block.id_map_address,
        "items": [{"class_id": i.class_id, "item_id": i.item_id, "entry": i.entry, "index": i.index,
                   "address": i.address, "size": i.size, "raw": i.raw} for i in block.items],
    }  # fmt: skip


def con_unit_limits(classes: Mapping[int, Any], support: Mapping[str, int] | None = None) -> ConUnitLimits:
    """:class:`ConUnitLimits` from parameter classes already read (no I/O): ``classes`` maps a class id to a block
    (:class:`iai_actuator_driver.info.ClassBlock`; classes 3, 7 and 9 are used), ``support`` the class-1 words (``SUP2``
    for CINP). Unset items (0 / FFFFFFFF) count as not known."""
    blocks = [{**_block(block), "class_id": ci} for ci, block in classes.items() if block is not None]
    words = None if support is None else {str(k): (None if v is None else int(v)) for k, v in support.items()}
    return ConUnitLimits(**_t.call("con_params.con_unit_limits", {"classes": blocks, "support": words}))


def min_positioning_band(
    family: str | int | None, lead_mm: float | None, encoder_pulses: int | None, *, mini_mm: float | None = None
) -> MinBand | None:
    """The smallest positioning band (No.10 and the rows' band) the unit accepts, in 0.01 mm, rounded up (the rule and
    its manual pages: above). ``None`` when neither the lead nor MINI is known; unknown
    ``encoder_pulses`` -> 800 assumed and ``certain`` False."""
    band = _t.call(
        "con_params.min_positioning_band",
        {"family": _family(family), "lead_mm": lead_mm, "encoder_pulses": encoder_pulses, "mini_mm": mini_mm},
    )
    return None if band is None else MinBand(band["mm"], band["formula"], band["doc"], band["certain"])


def check_value(
    p: ConParam, value: object, *, family: str | int | None = None, unit: ConUnitLimits | None = None
) -> ConCheck:
    """Parse and check a value for parameter ``p`` on a unit of ``family`` (no I/O). Raises ValueError (nothing to
    write) for a value outside the family range, above the unit's own maximum or, for No.10, below the minimum
    positioning band; documented minimums and cautions are ``warnings``, never refusals."""
    out = _t.call(
        "con_params.check_value",
        {"p": _json(p), "value": _t.value(value), "family": _family(family), "unit": _unit(unit)},
    )
    return ConCheck(out["value"], tuple(out["warnings"]), out["family"], out["lo"], out["hi"])


def audit(
    values: Mapping[int, object], *, family: str | int | None = None, unit: ConUnitLimits | None = None
) -> list[dict[str, Any]]:
    """Check the values a unit holds (``{number: decoded value}``) against the same rules as :func:`check_value` (no
    I/O). Returns one finding per value a write of it would refuse: ``number``, ``value``, ``problem``."""
    pairs = [[no, _t.value(v)] for no, v in values.items()]
    findings = _t.call("con_params.audit", {"values": pairs, "family": _family(family), "unit": _unit(unit)})
    return [{"number": f["number"], "value": values[f["number"]], "problem": f["problem"]} for f in findings]


def parameter_table(family: str | int | None = None) -> list[dict[str, Any]]:
    """The CON parameters of ``family`` as rows a GUI can show (no I/O): ``number``, ``name``, ``symbol``, ``item``,
    ``unit``, ``min`` / ``max`` (the family range), ``choices``, ``category`` / ``category_text``, ``factory_default``,
    ``dangerous``, ``writable``, ``note``, ``range_doc`` and ``verified_family`` (the driver has a range table for
    this family; False: the PCON-CB range is used)."""
    return _t.call("con_params.parameter_table", {"family": _family(family)})  # type: ignore[no-any-return]


def decode(p: ConParam, raw: int) -> int | float | str | None:
    """The value of ``p`` as the PC tool shows it, from the item's raw 32-bit value."""
    return _t.call("con_params.decode", {"p": _json(p), "raw": int(raw)})  # type: ignore[no-any-return]


def text(p: ConParam, value: object) -> str:
    """``value`` with its label or unit, e.g. ``"1:Forward"`` or ``"150.30 mm"``."""
    return _t.call("con_params.text", {"p": _json(p), "value": _t.value(value)})  # type: ignore[no-any-return]


def parse(p: ConParam, value: object, *, family: str | int | None = None) -> int | float | str:
    """A value for ``p`` (number, numeric string, a label such as ``"Forward"`` or ``"1:Forward"``, a dotted IPv4
    address for ``ipv4``). Raises ValueError for anything else or a value outside ``lo`` .. ``hi`` (the range of
    ``family`` when given). No.16 takes only :data:`CON_BAUD_RATES`."""
    return _t.call(  # type: ignore[no-any-return]
        "con_params.parse", {"p": _json(p), "value": _t.value(value), "family": _family(family)}
    )


def encode(p: ConParam, raw: int, value: int | float | str) -> int:
    """The new raw 32-bit item value for ``value`` (read-modify-write: bits outside the parameter are kept)."""
    return _t.call(  # type: ignore[no-any-return]
        "con_params.encode", {"p": _json(p), "raw": int(raw), "value": _t.value(value)}
    )
