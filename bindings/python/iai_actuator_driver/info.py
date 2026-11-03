"""Read-only unit information: everything an ELECYLINDER tells about itself, decoded where known.

Nothing here writes to the EC. The only function codes used are FC03 (read holding registers) and
FC11 (report slave id, through :meth:`Elecylinder.identify`). Every block read is bounded by the
EC's own directory (index table at 0x0000, :func:`iai_actuator_driver.registers.category_extent`): the EC
answers 0000 for unmapped addresses instead of an exception, so the directory is the only guard.

Entry points:

* :func:`unit_info` (also :meth:`Elecylinder.unit_info`): one JSON-serialisable dict with identity,
  user parameters, every raw parameter item, I/O images, decoded status words, monitor values,
  alarm (current + history), maintenance counters and position data. Schema in :func:`unit_info`.
* :func:`read_class`: one directory class by item ID (via the class's ID map), e.g. the parameter
  classes 3 / 4 / 7 / 8 / 9 / 0x0C or the maintenance class 0x200.
* :func:`read_block`: raw words of one directory block (for a register browser), clamped to it
  (a block of uncertain size up to its largest possible extent, :func:`readable_words`).
* :func:`decode_bits`: a status / I/O word decoded with one of the named bit tables below.
* :func:`parse_parameter_value` / :func:`encode_user_parameter`: pure helpers (no I/O) that
  :meth:`Elecylinder.write_user_parameter` uses to turn a parameter value, numbered or labelled as
  IA-OS shows it, into a 32-bit item.
* :func:`model_parameter_list` / :func:`annotate_parameters`: which user parameters IA-OS's screen
  lists for an FC11 unit code (the IA-OS lists per unit code) and which of them the driver may write.
* :func:`ec_parameter_guard` / :func:`smooth_accel_check`: the ME3816-6B rules for EC parameter writes and for
  moves with P5 (smooth accel/decel) on; :data:`EC_MANUAL_PARAMETERS` / :func:`ec_parameter_table`: the manual's
  categories (A / B / C), ranges and shipping defaults.
* :func:`input_ranges`: the input ranges per product line (EC, CON-CB, RCON, ROBONET) for a GUI.
* :func:`backup` / :func:`backup_axes`: a read-only JSON backup of identity, every parameter item, the position
  rows, the alarm history and the maintenance counters.

Evidence (see README "Evidence levels"): bit and item meanings marked ``confirmed`` were seen on
hardware (VERIFIED; ``VERIFIED-LIVE`` in the bit tables); the other names are IAI's register / bit
names (some lists were derived for another IAI model) or come from the CON manuals (``DOC-CON``),
and are labelled as UNVERIFIED on the EC.

On the core: the decoders run in the core, through ``iad_call_json`` (:mod:`._abi_tables`): the bit
tables (:data:`BIT_TABLES` and the ``*_BITS`` tables, read from the core once, at the first use), :func:`decode_bits`,
:func:`position_mode`, :func:`io_bit_tables`, :func:`rtc_in_use`, :func:`decode_maintenance` and
:func:`decode_alarm_history` (``core/src/info.cpp``). So do the readers:
:func:`unit_info`, :func:`backup` (the file is written here), :func:`read_directory`, :func:`read_block`,
:func:`read_class`, :func:`read_support_words` and :func:`read_accl` over ``iad_axis_info_call_json``, and
:func:`input_ranges`, :func:`product_line` and :func:`ec_parameter_table` over ``iad_call_json``. A reader given an axis
that is not a core axis (a test double) runs the shared Python reader.

Every other name is the shared module's function (``_shared/info.py``: the EC parameter tables and rules,
:func:`backup_axes`, the Python decoders), re-bound to this module: the names it calls are looked up here, so
``mock.patch("iai_actuator_driver.info.<name>")`` reaches the code that runs (:func:`backup_axes` calls the patched
:func:`backup`). Inside the core (a reader given a native axis) nothing is patched. The value types (:class:`BitDef`,
:class:`ClassItem`, :class:`ClassBlock`, :class:`ParamDef`) are the shared ones. ``tests/
test_tables_conformance.py`` compares the core's decoders and tables with those of 1.0
(``tests/tables_snapshot.json``).
"""

from __future__ import annotations

import ctypes
import dataclasses
import functools
import json
import sys
import types
from collections.abc import Iterable, Mapping, Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Any

from . import native as _n
from . import _shared
from ._shared import info as _source
from .values import IndexCategory, IndexHeader
from ._native import _abi_tables as _t

if TYPE_CHECKING:
    from .ec import Elecylinder


def _names(code: types.CodeType) -> set[str]:
    """The global names a function's code (and the code nested in it) reads."""
    found = set(code.co_names)
    for const in code.co_consts:
        if isinstance(const, types.CodeType):
            found |= _names(const)
    return found


# The bit tables are read from the core at the first use of one of them (native.py imports the driver modules while
# the library is being loaded, so nothing here calls the core at import).
_TABLE_NAMES = {
    "BIT_TABLES": None, "DSS1_BITS": "DSS1", "DSS2_BITS": "DSS2", "DSSE_BITS": "DSSE", "ZONS_BITS": "ZONS",
    "STAT_BITS": "STAT", "SSSE_BITS": "SSSE", "SIPM_BITS": "SIPM", "DRG1_BITS": "DRG1", "DRG2_BITS": "DRG2",
    "DRGE_BITS": "DRGE", "CTLF_BITS": "CTLF", "DOPM_BITS_3POS": "DOPM_3POS", "DOPM_BITS_2POS": "DOPM_2POS",
    "DIPM_BITS_3POS": "DIPM_3POS", "DIPM_BITS_2POS": "DIPM_2POS", "DIPM_BITS_SINGLE": "DIPM_SINGLE",
}  # fmt: skip


def _with_tables(function: types.FunctionType) -> Any:
    """A re-bound function that reads a bit table by its global name: the core's tables are loaded first."""

    @functools.wraps(function)
    def call(*args: Any, **kwargs: Any) -> Any:
        _bit_tables()
        return function(*args, **kwargs)

    return call


# The shared module's names first (re-bound; the tables are the core's); the decoders and readers below replace
# theirs.
for _name, _value in vars(_source).items():
    if _name.startswith("__") or _name in _TABLE_NAMES:
        continue
    if isinstance(_value, types.FunctionType) and _value.__globals__ is vars(_source):
        _value = _shared.rebound(_value, globals())
        if _names(_value.__code__) & _TABLE_NAMES.keys():
            _value = _with_tables(_value)
    globals()[_name] = _value
__all__ = list(_source.__all__)

BitDef = _source.BitDef
ClassItem = _source.ClassItem
ClassBlock = _source.ClassBlock
MAX_CLASS_WORDS = _source.MAX_CLASS_WORDS
PRODUCT_LINES = _source.PRODUCT_LINES

# the shared module's readers (re-bound): what a reader given an axis that is not a core axis runs
_PY_READERS: dict[str, Any] = {
    name: globals()[name]
    for name in ("unit_info", "backup", "read_directory", "read_block", "read_class", "read_support_words", "read_accl")
}


def _bit_def(d: Mapping[str, Any]) -> BitDef:
    return BitDef(d["bit"], d["name"], d["meaning"], d["confirmed"], d["evidence"], d["active_low"])


def _bit_tables() -> dict[str, tuple[BitDef, ...]]:
    """:data:`BIT_TABLES`, read from the core (with the ``*_BITS`` names) the first time."""
    if "BIT_TABLES" not in globals():
        tables = {name: tuple(_bit_def(d) for d in defs) for name, defs in _t.call("info.bit_tables").items()}
        globals().update({name: tables if key is None else tables[key] for name, key in _TABLE_NAMES.items()})
    return globals()["BIT_TABLES"]  # type: ignore[no-any-return]


def __getattr__(name: str) -> Any:
    if name in _TABLE_NAMES:
        _bit_tables()
        return globals()[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(_TABLE_NAMES))


def decode_bits(word: int, table: Sequence[BitDef] | str, *, width: int = 16) -> dict[str, Any]:
    """Decode ``word`` with a bit table (a sequence of :class:`BitDef` or a :data:`BIT_TABLES` key): ``{"value",
    "hex", "on": [names of set bits], "bits": [...], "unknown_set": [...]}``, every bit of the table high bit first,
    then any set bit the table does not name (``"b<n>"``). ``on`` is the raw bit (an ``active_low`` bit is 1 when
    normal). ValueError when the word does not fit in ``width`` bits."""
    if isinstance(table, str):
        _bit_tables()[table]  # a KeyError for a name that is not a table, as before
        defs: Any = table
    else:
        defs = [{"bit": d.bit, "name": d.name, "meaning": d.meaning, "confirmed": d.confirmed,
                 "evidence": d.evidence, "active_low": d.active_low} for d in table]  # fmt: skip
    return _t.call("info.decode_bits", {"word": word, "table": defs, "width": width})  # type: ignore[no-any-return]


def position_mode(code: int | None, model: str | None = None) -> dict[str, Any]:
    """PIO / position mode from user parameter 9 (class 8 item 2), else from the model string: ``code``, ``name``,
    ``positions``, ``layout`` (``"3pos"`` / ``"2pos"``, the DOPM / DIPM tables of :func:`io_bit_tables`), ``source``,
    ``confirmed``, ``note``."""
    return _t.call("info.position_mode", {"code": code, "model": model})  # type: ignore[no-any-return]


def io_bit_tables(mode: Mapping[str, Any] | str) -> dict[str, tuple[BitDef, ...]]:
    """``{"DIPM": table, "DOPM": table}`` for a :func:`position_mode` result (or a layout name)."""
    names = _t.call("info.io_bit_tables", {"mode": mode if isinstance(mode, str) else dict(mode)})
    tables = _bit_tables()
    return {"DIPM": tables[names["DIPM"]], "DOPM": tables[names["DOPM"]]}


def rtc_in_use(ssse: int | None) -> bool | None:
    """SSSE b8 (the unit's calendar is in use: time stamps are seconds since 2000-01-01), ``None`` when not read."""
    return _t.call("info.rtc_in_use", {"ssse": ssse})  # type: ignore[no-any-return]


def _block(block: ClassBlock) -> dict[str, Any]:
    return {
        "class_id": block.class_id, "address": block.address, "words": block.words, "member_size": block.member_size,
        "entries": block.entries, "id_map_address": block.id_map_address,
        "items": [{"class_id": i.class_id, "item_id": i.item_id, "entry": i.entry, "index": i.index,
                   "address": i.address, "size": i.size, "raw": i.raw} for i in block.items],
    }  # fmt: skip


def decode_maintenance(block: ClassBlock, *, rtc: bool | None = None, family: str | None = None) -> dict[str, Any]:
    """The maintenance class (12) as :func:`unit_info` reports it: the items with their meaning and, on a CON
    controller whose clock is in use (``rtc``), the TIMN calendar time (``family``: the product family)."""
    fam = family if family is None or isinstance(family, (int, str)) else str(family)
    return _t.call(  # type: ignore[no-any-return]
        "info.decode_maintenance", {"block": _block(block), "rtc": rtc, "family": fam}
    )


def decode_alarm_history(
    rec: IndexCategory,
    words: Sequence[int],
    current: int | None,
    dss1: int | None,
    c3: int | None,
    absolute_encoder: bool | None = None,
    *,
    rtc_in_use: bool | None = None,
    system_code: int | None = None,
) -> dict[str, Any]:
    """The alarm history as :func:`unit_info` reports it, from the class-5 record ``rec`` and its ``words``: the
    used slots (up to the first empty one) and the power-up markers, and the current alarm ``current`` (ALMC) with its
    level from DSS1 ``dss1`` (ALMH / ALML) and the class-3 code ``c3``; times are calendar times when ``rtc_in_use``."""
    record = {
        "category_id": rec.category_id, "model_code": rec.model_code, "member_count": rec.member_count,
        "list_count": rec.list_count, "data_address": rec.data_address, "id_list_address": rec.id_list_address,
        "coil_base": rec.coil_base, "member_size": rec.member_size,
    }  # fmt: skip
    return _t.call(  # type: ignore[no-any-return]
        "info.decode_alarm_history",
        {"record": record, "words": [int(w) for w in words], "current": current, "dss1": dss1, "c3": c3,
         "absolute_encoder": absolute_encoder, "rtc_in_use": rtc_in_use, "system_code": system_code},
    )


# =========================================================================== input ranges (no I/O)


def product_line(family: str | None, gateway: str | None = None) -> str:
    """The :data:`INPUT_RANGES` key for a unit: ``"RCON"`` / ``"ROBONET"`` behind those gateways, ``"EC"`` for an EC,
    ``"CON-CB"`` for every other family (a generic IAI controller included)."""
    return _t.call(  # type: ignore[no-any-return]
        "info.product_line", {"family": str(family) if family else None, "gateway": str(gateway) if gateway else None}
    )


def input_ranges(line: str | None = None, *, family: str | None = None) -> dict[str, Any]:
    """:data:`INPUT_RANGES` for one product ``line`` (or every line when ``None``) with the parameter rows filled in
    (:func:`ec_parameter_table` for the EC, :func:`iai_actuator_driver.con_params.parameter_table` for ``family`` on
    CON-CB). JSON-friendly; no I/O."""
    if line is not None and not isinstance(line, str):
        raise ValueError(f"unknown product line {line!r}; choose from {list(PRODUCT_LINES)}")
    return _t.call("info.input_ranges", {"line": line, "family": _t.value(family)})  # type: ignore[no-any-return]


def ec_parameter_table() -> list[dict[str, Any]]:
    """The EC user parameters as rows a GUI can show (no I/O): ``no``, ``name``, ``manual_name``, ``category`` /
    ``category_text``, ``unit``, ``manual_range``, ``shipping_default``, ``driver_range`` (the narrower limit the
    driver enforces, ``None`` when it has none), ``choices``, ``read_only``, ``home_parameter``, ``iaos_only`` and
    ``doc``. Plus No.143 (overload warning level), which the manual names but the driver does not map."""
    return _t.call("info.ec_parameter_table")  # type: ignore[no-any-return]


# =========================================================================== the readers (I/O)


def _core_axis(ec: Any) -> Any:
    """``ec`` when it is an axis of the native ``ec`` module (it has a core axis), else ``None``."""
    native_ec = sys.modules.get(f"{__package__}.ec")
    cls = getattr(native_ec, "Elecylinder", None)
    return ec if cls is not None and isinstance(ec, cls) else None


def _axis_json(ec: Any, name: str, *args: Any, cancellable: bool = False) -> Any:
    """The JSON output of a long read of ``ec``'s core axis. On the main thread the call runs on a helper thread, so
    Ctrl+C reaches Python: a ``cancellable`` read (unit_info, backup: the core reads them in the axis's cancel scope)
    ends at its next request (the cancel latch, cleared once it returned; nothing is sent for it), the others on their
    own timeouts. The output is fetched with a 64 KiB buffer, else from what the core kept for this call
    (iad_last_result_json, read on the thread that made it: no second read, and never another call's)."""
    from .ec import _error

    lib = ec._lib
    handle = ec._h()
    size = _n._JSON_START
    buffer = ctypes.create_string_buffer(size)
    required = ctypes.c_size_t(0)
    cancel = reset = None
    if cancellable and lib.has("iad_axis_cancel_latch"):
        cancel = functools.partial(_n._invoke, lib, "iad_axis_cancel_latch", handle)
        reset = functools.partial(_n._invoke, lib, "iad_axis_cancel_reset", handle)
    args = (handle, *args, buffer, size, ctypes.byref(required))
    note = f"iai_actuator_driver: {name} read cancelled on Ctrl+C (it ends at its next request; nothing is sent for it)"
    code, details, kept = _n._invoke_interruptible(lib, name, args, cancel, reset, call=_n._invoke_kept_buffered,
                                                   cancelled=note)  # fmt: skip
    if code == _n.IAD_ERR_BUFFER_TOO_SMALL:
        if kept is not None:
            return json.loads(kept)
        return json.loads(_n._read_text(lib, "iad_axis_last_result_json", handle))  # a library before 0.7
    if code != _n.IAD_OK:
        raise _error(code, details or {})
    return json.loads(buffer.value.decode("utf-8", errors="replace"))


def _info_call(ec: Any, function: str, args: Mapping[str, Any]) -> Any:
    """One reader of ``ec``'s core axis by name (iad_axis_info_call_json); unit_info and backup are cancellable."""
    return _axis_json(ec, "iad_axis_info_call_json", _n._utf8(function), json.dumps(args).encode("utf-8"),
                      cancellable=function in ("unit_info", "backup"))


def _records_json(records: Sequence[IndexCategory] | None) -> list[list[int]] | None:
    """The ``records=`` of a reader as the core takes them: ``None`` (the axis's directory) or 8-number records."""
    return None if records is None else [list(dataclasses.astuple(r)) for r in records]


def read_directory(ec: Elecylinder) -> tuple[IndexHeader, tuple[IndexCategory, ...]]:
    """Read the index table at 0x0000 (header, then LQNT records of DQNT words; FC03 only)."""
    axis = _core_axis(ec)
    if axis is None:
        return _PY_READERS["read_directory"](ec)  # type: ignore[no-any-return]
    out = _info_call(axis, "read_directory", {})
    return IndexHeader(**out["header"]), tuple(IndexCategory(*r) for r in out["records"])


def read_block(
    ec: Elecylinder,
    class_id: int,
    offset: int = 0,
    count: int | None = None,
    *,
    records: Sequence[IndexCategory] | None = None,
) -> list[int]:
    """Raw words of directory block ``class_id`` (FC03), ``count`` words from ``offset``, inside the block (its
    extent, or for a block of uncertain size the largest extent its record allows, :func:`readable_words`). An
    out-of-range request raises ValueError and nothing is read. ``count=None`` reads to the end of the certain
    extent."""
    axis = _core_axis(ec)
    if axis is None:
        return _PY_READERS["read_block"](ec, class_id, offset, count, records=records)  # type: ignore[no-any-return]
    args = {"class_id": class_id, "offset": offset, "count": count, "records": _records_json(records)}
    return _info_call(axis, "read_block", args)  # type: ignore[no-any-return]


def read_class(
    ec: Elecylinder,
    class_id: int,
    *,
    records: Sequence[IndexCategory] | None = None,
    max_words: int = MAX_CLASS_WORDS,
) -> ClassBlock:
    """Read directory class ``class_id`` and label every member with its item ID (FC03 only). A class without an ID
    map, or with mixed member sizes, is returned word by word with ``item_id = None``. Raises LookupError if the class
    is not in the directory and ValueError if it is larger than ``max_words``."""
    axis = _core_axis(ec)
    if axis is None:
        return _PY_READERS["read_class"](ec, class_id, records=records,  # type: ignore[no-any-return]
                                           max_words=max_words)  # fmt: skip
    args = {"class_id": class_id, "max_words": max_words, "records": _records_json(records)}
    b = _info_call(axis, "read_class", args)
    items = tuple(ClassItem(**i) for i in b["items"])
    return ClassBlock(b["class_id"], b["address"], b["words"], b["member_size"], b["entries"], b["id_map_address"],
                      items)  # fmt: skip


def read_support_words(ec: Elecylinder, records: Sequence[IndexCategory] | None = None) -> dict[str, int] | None:
    """Class 1 support-function words by name (SUPT, DIAG, MVPT, SPTS, CTYP, SUP2, ... SUP3). FC03 only. ``None``
    when the class is not in the directory. An exception response propagates (:class:`ModbusException`)."""
    axis = _core_axis(ec)
    if axis is None:
        return _PY_READERS["read_support_words"](ec, records)  # type: ignore[no-any-return]
    return _info_call(axis, "read_support_words", {"records": _records_json(records)})  # type: ignore[no-any-return]


def read_accl(ec: Elecylinder, records: Sequence[IndexCategory] | None = None) -> int | None:
    """ACCL (0x0612, u32; bits 8-11 = extensibility level) from the identification block. FC03 only."""
    axis = _core_axis(ec)
    if axis is None:
        return _PY_READERS["read_accl"](ec, records)  # type: ignore[no-any-return]
    return _info_call(axis, "read_accl", {"records": _records_json(records)})  # type: ignore[no-any-return]


def unit_info(ec: Elecylinder, *, sections: Iterable[str] | None = None) -> dict[str, Any]:
    """Everything the unit tells about itself, as one JSON-serialisable dict. Read-only (FC03 + FC11). ``sections``
    limits the work to some of :data:`SECTIONS` (default: all; ValueError for an unknown name)."""
    axis = _core_axis(ec)
    if axis is None:
        return _PY_READERS["unit_info"](ec, sections=sections)  # type: ignore[no-any-return]
    chosen = None if sections is None else list(sections)
    return _info_call(axis, "unit_info", {"sections": chosen})  # type: ignore[no-any-return]


def backup(ec: Elecylinder, path: str | Path | None = None, *, rows: str | Iterable[int] = "used") -> dict[str, Any]:
    """A read-only backup of one axis as a JSON-serialisable dict (FC03 + FC11 only; nothing is written to the unit),
    in the schema :data:`BACKUP_SCHEMA`. ``rows``: ``"used"``, ``"all"`` or row numbers (a selection the unit cannot
    serve is reported in ``errors``). ``path``: also written there as UTF-8 JSON (the dict then carries
    ``meta.path``). A unit that does not answer at all raises :class:`ModbusTimeout`; later read problems go to
    ``errors``."""
    axis = _core_axis(ec)
    if axis is None:
        return _PY_READERS["backup"](ec, path, rows=rows)  # type: ignore[no-any-return]
    chosen = rows if isinstance(rows, str) else sorted({int(n) for n in rows})
    out: dict[str, Any] = _info_call(axis, "backup", {"rows": chosen})
    if path is not None:
        target = Path(path)
        out["meta"]["path"] = str(target)
        target.write_text(json.dumps(out, indent=2), encoding="utf-8")
    return out
