"""The Python part of ``iai_actuator_driver.info`` (where its documentation is): the EC parameter tables and rules,
:func:`backup_axes`, the Python decoders and readers; that module re-binds these names (there the bit tables, the
decoders and the readers given a core axis are the core's).
"""

from __future__ import annotations

import datetime as _dt
import json
import math
import time
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from typing import TYPE_CHECKING, Any

from .. import con_params
from .. import registers as R
from ..alarms import (
    absolute_encoder_from_support,
    alarm_history_slots,
    alarm_info,
    alarm_is_fault,
    alarm_name,
    alarm_time,
    rtc_datetime,
)
from ..errors import EcDriverError, ModbusException, ModbusTimeout
from ..values import AlarmRecord, IndexCategory, IndexHeader, PositionEntry
from . import publish as _publish

if TYPE_CHECKING:
    from ..ec import Elecylinder

__all__ = [
    "BitDef",
    "BIT_TABLES",
    "DSS1_BITS",
    "DSS2_BITS",
    "DSSE_BITS",
    "ZONS_BITS",
    "STAT_BITS",
    "SSSE_BITS",
    "SIPM_BITS",
    "DRG1_BITS",
    "DRG2_BITS",
    "DRGE_BITS",
    "CTLF_BITS",
    "DOPM_BITS_3POS",
    "DOPM_BITS_2POS",
    "DIPM_BITS_3POS",
    "DIPM_BITS_2POS",
    "DIPM_BITS_SINGLE",
    "ClassItem",
    "ClassBlock",
    "ParamDef",
    "USER_PARAMETERS",
    "MAINTENANCE_ITEMS",
    "SECTIONS",
    "decode_bits",
    "decode_monitor",
    "io_bit_tables",
    "position_mode",
    "read_directory",
    "decode_alarm_history",
    "decode_maintenance",
    "directory_summary",
    "block_extent",
    "readable_words",
    "read_block",
    "read_class",
    "decode_user_parameters",
    "decode_user_parameter",
    "USER_PARAMETERS_BY_NO",
    "HOME_PARAMETERS",
    "PARAMETER_CLASSES_NAMED",
    "PARAM_RAM_ONLY_NOTE",
    "PARAM_COMMIT_NOTE",
    "PARAM_HOME_WARNING",
    "NOT_LOCATED_PARAMETERS",
    "model_parameter_list",
    "parameter_visibility",
    "p9_valid_values",
    "extensibility_level",
    "home_pattern_settable",
    "MULTI_IO_PAIRS",
    "READ_ONLY_PARAMETERS",
    "VISIBILITY_CLASSES",
    "decode_support_words",
    "SUPPORT_BITS",
    "needs_model_opt_in",
    "MODEL_SPECIFIC_KINDS",
    "OPT_IN_NOTE",
    "annotate_parameters",
    "parameter_choices",
    "parse_parameter_value",
    "encode_user_parameter",
    "unit_info",
    "EC_CATEGORY_TEXT",
    "EC_MANUAL_PARAMETERS",
    "ParameterRefused",
    "SMOOTH_ACCEL_LIMIT_S",
    "find_user_parameter",
    "has_acr_option",
    "ramp_time_s",
    "smooth_accel_check",
    "ec_parameter_guard",
    "ec_parameter_table",
    "annotate_con_parameters",
    "con_range_check",
    "PRODUCT_LINES",
    "INPUT_RANGES",
    "product_line",
    "input_ranges",
    "TIMN_ADDRESS",
    "rtc_in_use",
    "BACKUP_SCHEMA",
    "backup",
    "backup_axes",
]

SCHEMA = "ec_driver.unit_info/1"
SECTIONS = ("identity", "parameters", "monitor", "alarms", "maintenance", "positions")
PARAMETER_CLASSES = (0x0003, 0x0004, 0x0007, 0x0008, 0x0009, 0x000C)
MAX_CLASS_WORDS = 1024  # read_class guard: the position table (12288 words) is read row by row instead

# =========================================================================== bit tables


@dataclass(frozen=True)
class BitDef:
    """One named bit of a status / I/O word.

    ``confirmed`` = the meaning was seen on hardware (VERIFIED-LIVE); otherwise ``evidence`` says
    where the name comes from. ``active_low`` bits (e.g. ``*ALM``) are 1 in the normal state.
    """

    bit: int
    name: str
    meaning: str = ""
    confirmed: bool = False
    evidence: str = "IAI bit name; meaning UNVERIFIED on the EC"
    active_low: bool = False


_LIVE = "VERIFIED-LIVE"
_VALUE = "value seen live; meaning UNVERIFIED"
_DERIVED = "IAI bit name from a list for another IAI model; meaning UNVERIFIED on the EC"
_DOC_EC = "EC manuals ME3816 / ME3837 / ME0384 (PIO); bit order in this word unconfirmed (PIO not wired)"
_DOC_EC_RES = "EC manuals ME3816 / ME3837 / ME0384 (PIO); bit 2 not yet seen live (RES was not pressed)"
_PIO_LIVE = (f"{_LIVE} (EC-GRBP8M gripper, 2026-10-30: DIPM read while the inputs were pressed by hand, no program "
             "holding PC control; 0x0001 = ST0, 0x0002 = ST1, the unit moved with them)")


def _bits(*defs: BitDef) -> tuple[BitDef, ...]:
    return tuple(sorted(defs, key=lambda d: -d.bit))


DSS1_BITS = _bits(
    BitDef(15, "EMGS", "emergency stop", evidence="DOC-CON (always 0: the IAI PC cable shorts EMG)"),
    BitDef(14, "SFTY", "safety speed on (mirrors DRG1 b14; caps ~100 mm/s)", True, _LIVE),
    BitDef(13, "PWR", "controller ready (always 1)", evidence=_VALUE),
    BitDef(12, "SV", "servo on", True, _LIVE),
    BitDef(11, "PSFL", "push missed (no contact over the band)", True, _LIVE),
    BitDef(10, "ALMH", "alarm, operation-cancel level (servo off)", True, f"{_LIVE} (gripper, alarm 0x0EE)"),
    BitDef(9, "ALML", "message-level alarm / warning", evidence="DOC-CON"),
    BitDef(8, "ABER", "absolute error", evidence="DOC-CON"),
    BitDef(7, "BKRL", "brake released (1 whenever the servo is on)", True, _LIVE),
    BitDef(6, "TQLM", "torque limit"),
    BitDef(5, "STP", "pause (STP) held", True, _LIVE),
    BitDef(4, "HEND", "home established (0 only during a home return / after home loss)", True, _LIVE),
    BitDef(3, "PEND", "in position: inside the INP band (also after STOP, push contact, homing)", True, _LIVE),
    BitDef(2, "CEND", "load-cell bit (never set)"),
    BitDef(1, "CLBS", "load-cell bit (never set)"),
    BitDef(0, "MEND", "move command finished (also after a push miss: check PSFL)", True, _LIVE),
)

DSS2_BITS = _bits(
    BitDef(15, "ENBS", "enable (always 1)", evidence=_VALUE),
    BitDef(14, "TSFC"),
    BitDef(13, "LOAD"),
    BitDef(12, "TRQS", "push current at its limit (contact push)", True, f"{_LIVE} (EC-DS4H slider, seen once)"),
    BitDef(11, "MODS", "teaching mode", evidence="DOC-CON"),
    BitDef(10, "TEAC", "teach command", evidence="DOC-CON"),
    BitDef(9, "JOG+", "JOG+ held", True, f"{_LIVE} (EC-DS4H slider, seen once)"),
    BitDef(8, "JOG-", "JOG- held", evidence="DOC-CON"),
    *(BitDef(n, f"PE{n}", f"position {n} complete", evidence="DOC-CON") for n in (7, 6, 5, 4, 3, 1, 0)),
    BitDef(2, "PE2", "position 2 complete (after a 0x9800 = 2 move)", True, f"{_LIVE} (EC-DS4H slider, seen once)"),
)

DSSE_BITS = _bits(
    BitDef(15, "EMGP", "emergency stop input", evidence="DOC-CON"),
    BitDef(14, "MPUV", "motor drive power low / cut off", evidence="DOC-CON"),
    BitDef(13, "RMDS", "MANU mode (always 1)", evidence=_VALUE),
    BitDef(12, "SSON", "1 with the servo on", evidence=_VALUE),
    BitDef(11, "GHMS", "home return in progress", True, _LIVE),
    BitDef(10, "PUSH", "pushing / holding after contact", True, _LIVE),
    BitDef(9, "PSNS", "excitation detection complete (0 during the homing creep)", evidence=_VALUE),
    BitDef(8, "PMSS", "Modbus has control (PMSL); 0 after power-up", True, _LIVE),
    BitDef(7, "PARW", "1 = parameter write idle / complete (the parameter commit handshake)",
           evidence="value seen live (1 when idle); meaning UNVERIFIED"),
    BitDef(6, "POSW", "always 1", evidence=_VALUE),
    BitDef(5, "MOVE", "profile generator running (clears before standstill)", True, _LIVE),
    BitDef(4, "DST1", "motion phase? (set while cruising)", evidence=_VALUE),
    BitDef(3, "DST0", "motion phase? (first sample after a command)", evidence=_VALUE),
    BitDef(2, "REND", "never set"),
    BitDef(1, "PCRF", "always 1", evidence=_VALUE),
    BitDef(0, "PHDT", "holding after push contact", True, f"{_LIVE} (EC-DS4H slider)"),
)

ZONS_BITS = _bits(
    BitDef(15, "LS3", "LS of position 3", evidence=_DERIVED),
    BitDef(14, "LS2", "within the LS range of row 2 (middle)", True, _LIVE),
    BitDef(13, "LS1", "within the LS range of row 1 (forward end; a push row: its push end)", True, _LIVE),
    BitDef(12, "LS0", "within the LS range of row 0 (backward end)", True, _LIVE),
    BitDef(8, "ZP", evidence=_DERIVED),
    BitDef(1, "Z2", "inside zone 2 (spans the whole stroke on both units)", True, _LIVE),
    BitDef(0, "Z1", "inside zone 1 (spans the whole stroke on both units)", True, _LIVE),
)

_STAT_NAMES = {
    31: "BATL", 30: "OVLW", 29: "RVLF", 28: "SSTO", 27: "EECP", 26: "DSLC", 25: "MTRG", 24: "BOLW",
    23: "PRTC", 22: "CLDC", 21: "MONI", 20: "AABS", 19: "SFD", 18: "PLOS", 17: "ASOF", 16: "AEEP",
    15: "CARS", 14: "ZENB", 13: "ASEN", 12: "FSRS", 8: "FAN", 6: "LRUN", 5: "LINK", 4: "RMDS",
    3: "HEND", 2: "SV", 1: "SON", 0: "MPOW",
}  # fmt: skip
STAT_BITS = _bits(*(BitDef(b, n, evidence=_DERIVED) for b, n in _STAT_NAMES.items()))

_SSSE_NAMES = {
    15: "RENB", 14: "AIRC", 13: "PTEP", 12: "ABUE", 11: "ALMC", 8: "URTC", 7: "CL3", 6: "CL2", 5: "CL1",
    4: "CL0", 3: "CM8", 2: "CM4", 1: "CM2", 0: "CM1",
}  # fmt: skip
SSSE_BITS = _bits(*(BitDef(b, n, evidence=_DERIVED) for b, n in _SSSE_NAMES.items()))

_SIPM_NAMES = {
    15: "SWJ-", 14: "NP", 13: "SWJ+", 12: "PP", 11: "SDWN", 10: "FANE", 9: "PTSW", 8: "MDSW", 7: "ENBL",
    6: "RSV1", 5: "RSV0", 4: "BLCT", 3: "HMCK", 2: "OT", 1: "CREP", 0: "LS",
}  # fmt: skip
SIPM_BITS = _bits(*(BitDef(b, n, evidence=_DERIVED) for b, n in _SIPM_NAMES.items()))

DRG1_BITS = _bits(
    BitDef(15, "EMG", "software emergency stop", evidence="DOC-CON"),
    BitDef(14, "SFTY", "safety-speed command", True, f"{_LIVE} (readback 0x5000 with SFTY)"),
    BitDef(12, "SON", "servo-on command", True, f"{_LIVE} (readback 0x1000)"),
    BitDef(11, "CSTP"),
    BitDef(8, "ALRS", "alarm reset", evidence="DOC-CON"),
    BitDef(7, "BKRL", "forced brake release", evidence="DOC-CON"),
    BitDef(6, "TQLM"),
    BitDef(5, "STP", "pause", evidence="DOC-CON"),
    BitDef(4, "HOME", "home return", evidence="DOC-CON"),
    BitDef(3, "CSTR", "start move to POSR", evidence="DOC-CON"),
)

DRG2_BITS = _bits(
    BitDef(15, "JVEL"), BitDef(14, "JISL"), BitDef(13, "EXTA"), BitDef(12, "PMOD"), BitDef(11, "MOD"),
    BitDef(10, "TEAC", "teach (writes the position table)", evidence="DOC-CON"),
    BitDef(9, "JOG+", "jog + (runs at ~100 mm/s; never used by the driver)", True, _LIVE),
    BitDef(8, "JOG-", "jog -", evidence="DOC-CON"),
    *(BitDef(n, f"ST{n}", f"start input {n} (solenoid-valve mode)", evidence="DOC-CON") for n in range(7, -1, -1)),
)  # fmt: skip

DRGE_BITS = _bits(
    BitDef(15, "SFRS"), BitDef(14, "BRCG"), BitDef(13, "RMOD"), BitDef(12, "DCLR"), BitDef(11, "DSTR"),
    BitDef(10, "WAIT"), BitDef(9, "CLBR"),
    BitDef(8, "PMSL", "Modbus control requested (DSSE PMSS follows)", True, f"{_LIVE} (readback 0x0100)"),
    BitDef(7, "PARW"), BitDef(4, "MIWE"),
    BitDef(3, "STOP", "decel stop", True, _LIVE),
    BitDef(2, "RSTR"), BitDef(1, "PCRF"),
)  # fmt: skip

CTLF_BITS = _bits(
    BitDef(15, "PRG"), BitDef(14, "SPOS"), BitDef(11, "PTOF"), BitDef(10, "SMOD"), BitDef(9, "ASO1"),
    BitDef(8, "ASO0"), BitDef(7, "MOD1"), BitDef(6, "MOD0"),
    BitDef(5, "GSL1", "row role bit (0x0020 on the slider's row 2); meaning unknown", evidence=_VALUE),
    BitDef(4, "GSL0", "row role bit (0x0010 on the slider's row 1); meaning unknown", evidence=_VALUE),
    BitDef(3, "INC", "relative move: PCMD is a signed distance", True, _LIVE),
    BitDef(2, "DIR", "push direction: 1 = away from home, 0 = toward home", True, _LIVE),
    BitDef(1, "PUSH", "push move over the INP band at PPOW", True, _LIVE),
)  # fmt: skip

# PIO images. DOPM layouts VERIFIED-LIVE (3POS on the EC-DS4H slider, 2POS on the EC-GRBP8M gripper). DIPM: ST0 = bit 0
# and ST1 = bit 1 of the 2POS layout VERIFIED-LIVE on the gripper (inputs pressed by hand); RES (bit 2) was not pressed
# and the other layouts were never seen (no PIO wired on the slider): DOC only.
DOPM_BITS_3POS = _bits(
    BitDef(0, "LS0", "backward end (row 0) within the LS range / PE0", True, f"{_LIVE} (0x0009 at 0.00 mm)"),
    BitDef(1, "LS1", "forward end (row 1) within the LS range / PE1", True, f"{_LIVE} (0x000A at 150.00 mm)"),
    BitDef(2, "LS2", "middle (row 2) within the LS range", True, f"{_LIVE} (0x000C at 75.00 mm)"),
    BitDef(3, "*ALM", "alarm output, active low: 1 = no alarm", True, f"{_LIVE} (seen 1 only)", active_low=True),
)
DOPM_BITS_2POS = _bits(
    BitDef(0, "LS0", "backward end (row 0) within the LS range / PE0 push complete", True, f"{_LIVE} (gripper 0x0005)"),
    BitDef(1, "LS1", "forward end (row 1) / PE1; a push row's bit stays 0 after a missed push", True,
           f"{_LIVE} (gripper 0x0006 at 9.50 mm, 0x0004 after a miss)"),
    BitDef(2, "*ALM", "alarm output, active low: 1 = no alarm", True, f"{_LIVE} (seen 1 only)", active_low=True),
)  # fmt: skip
DIPM_BITS_3POS = _bits(
    BitDef(0, "ST0", "go to the backward end (ST0 + ST1 = middle)", evidence=_DOC_EC),
    BitDef(1, "ST1", "go to the forward end (ST0 + ST1 = middle)", evidence=_DOC_EC),
    BitDef(3, "RES", "alarm reset", evidence=_DOC_EC),
)
DIPM_BITS_2POS = _bits(
    BitDef(0, "ST0", "go to the backward end (starts homing if home is not established)", True, _PIO_LIVE),
    BitDef(1, "ST1", "go to the forward end", True, _PIO_LIVE),
    BitDef(2, "RES", "alarm reset", evidence=_DOC_EC_RES),
)
DIPM_BITS_SINGLE = _bits(
    BitDef(0, "FWBW", "single solenoid: ON = forward, OFF = backward", evidence=_DOC_EC),
    BitDef(2, "RES", "alarm reset", evidence=_DOC_EC),
)

BIT_TABLES: dict[str, tuple[BitDef, ...]] = {
    "DSS1": DSS1_BITS,
    "DSS2": DSS2_BITS,
    "DSSE": DSSE_BITS,
    "ZONS": ZONS_BITS,
    "STAT": STAT_BITS,
    "SSSE": SSSE_BITS,
    "SIPM": SIPM_BITS,
    "DRG1": DRG1_BITS,
    "DRG2": DRG2_BITS,
    "DRGE": DRGE_BITS,
    "CTLF": CTLF_BITS,
    "DOPM_3POS": DOPM_BITS_3POS,
    "DOPM_2POS": DOPM_BITS_2POS,
    "DIPM_3POS": DIPM_BITS_3POS,
    "DIPM_2POS": DIPM_BITS_2POS,
    "DIPM_SINGLE": DIPM_BITS_SINGLE,
}


def decode_bits(word: int, table: Sequence[BitDef] | str, *, width: int = 16) -> dict[str, Any]:
    """Decode ``word`` with a bit table (a sequence of :class:`BitDef` or a :data:`BIT_TABLES` key).

    Returns ``{"value", "hex", "on": [names of set bits], "bits": [...], "unknown_set": [...]}``: ``bits``
    lists every bit of the table, high bit first, as ``{"bit", "name", "on", "meaning", "confirmed",
    "evidence", "active_low"}``, followed by any set bit the table does not name (as ``"b<n>"``, which
    is also listed in ``unknown_set``). ``on`` is the raw bit; for an ``active_low`` bit 1 is normal.
    """
    defs = BIT_TABLES[table] if isinstance(table, str) else table
    if not 0 <= word < (1 << width):
        raise ValueError(f"{word} does not fit in {width} bits")
    named = {d.bit for d in defs}
    bits = [
        {
            "bit": d.bit,
            "name": d.name,
            "on": bool(word >> d.bit & 1),
            "meaning": d.meaning,
            "confirmed": d.confirmed,
            "evidence": d.evidence,
            "active_low": d.active_low,
        }
        for d in sorted(defs, key=lambda d: -d.bit)
    ]
    unknown = [b for b in range(width - 1, -1, -1) if word >> b & 1 and b not in named]
    bits += [
        {"bit": b, "name": f"b{b}", "on": True, "meaning": "", "confirmed": False, "evidence": "not named",
         "active_low": False}  # fmt: skip
        for b in unknown
    ]
    return {
        "value": word,
        "hex": f"0x{word:0{width // 4}X}",
        "on": [b["name"] for b in bits if b["on"]],
        "bits": bits,
        "unknown_set": unknown,
    }


# =========================================================================== position mode


def position_mode(code: int | None, model: str | None = None) -> dict[str, Any]:
    """PIO / position mode from user parameter 9 (class 8 item 2), else from the model string.

    0 = Double solenoid and 3 = Single solenoid (2 positions) are VERIFIED on the gripper (set in
    IA-OS, registers compared); 4 is what the 3-position slider holds (option MF), so 4 = 3-position
    is inferred from one unit. ``layout`` is ``"3pos"`` or ``"2pos"`` (the DOPM / DIPM tables of :func:`io_bit_tables`).
    """
    if code == 4:
        return {"code": 4, "name": "3-position", "positions": 3, "layout": "3pos", "source": "user parameter 9 = 4",
                "confirmed": False, "note": "4 seen on the 3-position slider only (model option MF)"}  # fmt: skip
    if code == 0:
        return {"code": 0, "name": "2-position (double solenoid)", "positions": 2, "layout": "2pos",
                "source": "user parameter 9 = 0", "confirmed": True, "note": ""}  # fmt: skip
    if code == 3:
        return {"code": 3, "name": "2-position (single solenoid)", "positions": 2, "layout": "2pos",
                "source": "user parameter 9 = 3", "confirmed": True, "note": ""}  # fmt: skip
    mf = bool(model) and ("-MF" in model.upper() or model.upper().endswith("MF"))  # type: ignore[union-attr]
    if code is None and mf:
        return {"code": None, "name": "3-position", "positions": 3, "layout": "3pos", "source": "model string option MF",
                "confirmed": False, "note": "user parameter 9 not readable"}  # fmt: skip
    why = "user parameter 9 not readable" if code is None else f"unknown user parameter 9 value {code}"
    return {"code": code, "name": "2-position (assumed)", "positions": 2, "layout": "2pos", "source": "assumed",
            "confirmed": False, "note": why}  # fmt: skip


def io_bit_tables(mode: Mapping[str, Any] | str) -> dict[str, tuple[BitDef, ...]]:
    """``{"DIPM": table, "DOPM": table}`` for a :func:`position_mode` result (or a layout name)."""
    layout = mode if isinstance(mode, str) else mode.get("layout")
    if layout == "3pos":
        return {"DIPM": DIPM_BITS_3POS, "DOPM": DOPM_BITS_3POS}
    single = not isinstance(mode, str) and mode.get("code") == 3
    return {"DIPM": DIPM_BITS_SINGLE if single else DIPM_BITS_2POS, "DOPM": DOPM_BITS_2POS}


# =========================================================================== directory


def read_directory(ec: Elecylinder) -> tuple[IndexHeader, tuple[IndexCategory, ...]]:
    """Read the index table at 0x0000 (header, then LQNT records of DQNT words; FC03 only)."""
    head = ec.read_registers(R.INDEX_HEADER, R.INDEX_HEADER_WORDS)
    header = IndexHeader(*head[:5])
    if header.words_per_record != 8 or not 0 < header.record_count <= 64:
        raise ValueError(f"index table: unexpected layout {header}")
    words = ec.read_registers(R.INDEX_RECORDS, 8 * header.record_count)
    return header, tuple(IndexCategory(*words[i : i + 8]) for i in range(0, len(words), 8))


def _records(ec: Elecylinder, records: Sequence[IndexCategory] | None) -> Sequence[IndexCategory]:
    if records is not None:
        return records
    return ec.categories or read_directory(ec)[1]


def _record(records: Sequence[IndexCategory], class_id: int) -> IndexCategory:
    rec = next((c for c in records if c.category_id == class_id), None)
    if rec is None:
        raise LookupError(f"class 0x{class_id:04X} is not in this unit's directory")
    return rec


def block_extent(record: IndexCategory) -> tuple[int, bool, int]:
    """``(words, exact, words_max)`` of a directory record; see :func:`iai_actuator_driver.registers.category_extent`."""
    return R.category_extent(record.category_id, record.member_count, record.list_count, record.member_size)


def readable_words(record: IndexCategory, records: Sequence[IndexCategory] = ()) -> int:
    """How many words of a block a read-only browser may read: its extent, or for a block of uncertain
    size (``exact`` False) the largest extent its record allows (``words_max``), never into the next
    directory block of ``records``. Words past :func:`block_extent` ``words`` are "size uncertain"."""
    words, exact, words_max = block_extent(record)
    if exact:
        return words
    following = [r.data_address for r in records if r.data_address > record.data_address]
    if following:
        words_max = min(words_max, min(following) - record.data_address)
    return max(words, words_max)


def directory_summary(header: IndexHeader | None, records: Sequence[IndexCategory]) -> dict[str, Any]:
    """JSON-friendly directory: header fields and one entry per record with its register extent."""
    rows = []
    for rec in records:
        words, exact, words_max = block_extent(rec)
        rows.append(
            {
                "class_id": rec.category_id,
                "class": f"0x{rec.category_id:04X}",
                "name": R.CATEGORY_NAMES.get(rec.category_id, "?"),
                "address": f"0x{rec.data_address:04X}",
                "end": f"0x{rec.data_address + words - 1:04X}",
                "words": words,
                "exact": exact,
                "words_max": words_max,
                "words_readable": readable_words(rec, records),  # what the register browser may read
                "members": rec.member_count,
                "entries": rec.list_count,
                "member_size": rec.member_size,
                "id_map": f"0x{rec.id_list_address:04X}" if rec.id_list_address else None,
                "coil_base": f"0x{rec.coil_base:04X}" if rec.coil_base else None,
                "flags": f"0x{rec.model_code:04X}",
            }
        )
    return {
        "header": None
        if header is None
        else {
            "model_code": header.model_code,
            "words_per_record": header.words_per_record,
            "record_count": header.record_count,
            "rx_buffer": header.rx_buffer,
            "tx_buffer": header.tx_buffer,
        },
        "record_count": len(rows),
        "total_words": sum(r["words"] for r in rows),
        "records": rows,
    }


def read_block(
    ec: Elecylinder,
    class_id: int,
    offset: int = 0,
    count: int | None = None,
    *,
    records: Sequence[IndexCategory] | None = None,
) -> list[int]:
    """Raw words of directory block ``class_id`` (FC03), ``count`` words from ``offset``.

    The read must stay inside the block: its extent, or for a block of uncertain size (mixed member
    sizes not in :data:`iai_actuator_driver.registers.KNOWN_MIXED_EXTENTS`) the largest extent its record
    allows (:func:`readable_words`; words past :func:`block_extent` ``words`` are "size uncertain").
    An out-of-range request raises ValueError and nothing is read. ``count=None`` reads to the end
    of the certain extent.
    """
    all_records = _records(ec, records)
    rec = _record(all_records, class_id)
    words, readable = block_extent(rec)[0], readable_words(rec, all_records)
    if count is None:
        count = words - offset
    if offset < 0 or count < 1 or offset + count > readable:
        size = f"{words} registers" if readable == words else f"{words}-{readable} registers (size uncertain)"
        raise ValueError(
            f"class 0x{class_id:04X} has {size} at 0x{rec.data_address:04X}: "
            f"offset {offset} + count {count} is outside it"
        )
    return ec.read_registers(rec.data_address + offset, count)


# =========================================================================== classes by item id


@dataclass(frozen=True)
class ClassItem:
    """One member of a directory class (16-bit for MS = 1, 32-bit high word first for MS = 2)."""

    class_id: int
    item_id: int | None
    entry: int
    index: int
    address: int
    size: int
    raw: int

    @property
    def signed(self) -> int:
        bits = 16 * self.size
        return self.raw - (1 << bits) if self.raw >> (bits - 1) & 1 else self.raw

    @property
    def hex(self) -> str:
        return f"{self.raw:0{4 * self.size}X}"

    @property
    def key(self) -> str:
        """``"<class>.<item>"`` (class in hex above 9), plus ``[entry]`` for multi-entry classes."""
        cls = f"{self.class_id:X}" if self.class_id > 9 else str(self.class_id)
        item = "?" if self.item_id is None else str(self.item_id)
        return f"{cls}.{item}" + (f"[{self.entry}]" if self.entry else "")

    def to_dict(self) -> dict[str, Any]:
        return {
            "key": self.key,
            "id": self.item_id,
            "entry": self.entry,
            "index": self.index,
            "address": f"0x{self.address:04X}",
            "words": self.size,
            "hex": self.hex,
            "unsigned": self.raw,
            "signed": self.signed,
            "scaled": round(self.signed * 0.01, 2),  # x 0.01: the EC's usual mm / mm/s / G unit (a convenience)
        }


@dataclass(frozen=True)
class ClassBlock:
    """A directory class read by :func:`read_class`."""

    class_id: int
    address: int
    words: int
    member_size: int
    entries: int
    id_map_address: int | None
    items: tuple[ClassItem, ...]

    def item(self, item_id: int, entry: int = 0) -> ClassItem | None:
        """The item with this ID (``None`` if this unit's class has no such item)."""
        return next((i for i in self.items if i.item_id == item_id and i.entry == entry), None)

    def to_dict(self) -> dict[str, Any]:
        return {
            "class_id": self.class_id,
            "class": f"0x{self.class_id:04X}",
            "name": R.CATEGORY_NAMES.get(self.class_id, "?"),
            "address": f"0x{self.address:04X}",
            "words": self.words,
            "member_size": self.member_size,
            "entries": self.entries,
            "id_map": None if self.id_map_address is None else f"0x{self.id_map_address:04X}",
            "items": [i.to_dict() for i in self.items],
        }


def read_class(
    ec: Elecylinder,
    class_id: int,
    *,
    records: Sequence[IndexCategory] | None = None,
    max_words: int = MAX_CLASS_WORDS,
) -> ClassBlock:
    """Read directory class ``class_id`` and label every member with its item ID (FC03 only).

    The data (DQ x MS x LQ registers at DA) and the ID map ((item id, 0) pairs at IA) come from the
    EC's own directory, so the same parameter is found by ID whatever its address on this model
    (class 8 item 50 is 0x083A on the gripper, 0x083E on the slider). A class without an ID map,
    or with mixed member sizes (MS = 0), is returned word by word with ``item_id = None``. Raises
    LookupError if the class is not in the directory and ValueError if it is larger than
    ``max_words`` (the position table: use :meth:`Elecylinder.read_position_table`).
    """
    rec = _record(_records(ec, records), class_id)
    words, _, _ = block_extent(rec)
    if words > max_words:
        raise ValueError(f"class 0x{class_id:04X} has {words} registers (> {max_words}); read it in parts")
    data = ec.read_registers(rec.data_address, words) if words else []
    size = rec.member_size if rec.member_size in (1, 2) else 1
    per_entry = rec.member_count * size if rec.member_size in (1, 2) else words
    ids: list[int] | None = None
    if rec.id_list_address and rec.member_size in (1, 2) and rec.member_count:
        id_words = ec.read_registers(rec.id_list_address, 2 * rec.member_count)
        ids = id_words[0::2]
    items = []
    for n in range(0, len(data), size):
        entry, index = divmod(n, per_entry)
        index //= size
        raw = data[n] if size == 1 else (data[n] << 16) | data[n + 1]
        item_id = ids[index] if ids is not None and index < len(ids) else None
        items.append(ClassItem(class_id, item_id, entry, index, rec.data_address + n, size, raw))
    return ClassBlock(
        class_id, rec.data_address, words, rec.member_size, rec.list_count, rec.id_list_address or None, tuple(items)
    )


# =========================================================================== user parameters


@dataclass(frozen=True)
class ParamDef:
    """A user parameter (numbered as IA-OS shows it) and where it lives (class.item; kind ``mm`` / ``bit`` /
    ``parity`` / ``enum`` / ``multi_io``). ``read_only`` parameters are decoded but never written by this driver;
    ``entry`` picks a multi-entry class's entry (zone sets); ``item_id2`` is the second item of a pair (P11).

    The manual's view (ME3816-6B p.6-6, filled in from :data:`EC_MANUAL_PARAMETERS`): ``category`` (A = check or
    change for the use, B = use as shipped, C = set at shipping for the EC's specification; :data:`EC_CATEGORY_TEXT`),
    ``manual_name`` (also accepted as an alias by :func:`find_user_parameter`), ``manual_range``,
    ``shipping_default`` and ``doc`` (pages). ``iaos_only``: not in ME3816-6B (P11-P18); the PC tool lists the
    parameter, the manual does not."""

    no: int
    name: str
    class_id: int
    item_id: int
    kind: str
    unit: str = ""
    bit: int | None = None
    offset_mm: float = 0.0
    choices: Mapping[int, str] | None = None
    encoding: str = ""
    read_only: bool = False
    entry: int = 0
    item_id2: int | None = None
    evidence: str = ""
    note: str = ""
    category: str = ""
    manual_name: str = ""
    manual_range: str = ""
    shipping_default: str = ""
    doc: str = ""
    iaos_only: bool = False


_ENABLE = {0: "0:Disable", 1: "1:Enable"}
_PARAM_EVIDENCE = ("VERIFIED on the EC-GRBP8M gripper (fw 0002): changed one at a time in IA-OS and the registers "
                   "compared; same item IDs assumed elsewhere")
_UNVERIFIED_ITEM = "UNVERIFIED: location expected, never tested on hardware (IA-OS hid it on all units tested)"
_READ_ONLY_UNVERIFIED = "read-only in this driver: location expected, never tested on hardware"
# P11 "Multi I/O function" (UNVERIFIED): the pair (byte 1 of FI00 8.25, byte 1 of FO00 8.29) -> the number IA-OS shows.
MULTI_IO_PAIRS: dict[tuple[int, int], int] = {
    (0, 0): 0, (15, 15): 1, (13, 0): 2, (0, 18): 3, (0, 32): 4, (0, 87): 5, (22, 28): 6, (26, 43): 7,
}  # fmt: skip

# P1-P7, P9, P10, P12: VERIFIED on the EC-GRBP8M gripper (fw 0002) by changing one parameter at a time in IA-OS and
# comparing the registers by class + item ID; the same items are assumed on every EC family. P3 decodes by the
# PARITY of the ORG home pattern; P8, P11, P13-P18 are UNVERIFIED locations and read-only here.
USER_PARAMETERS: tuple[ParamDef, ...] = (
    ParamDef(1, "Operation range adjustment", 7, 1, "mm", "mm", offset_mm=-0.30,
             encoding="s32 0.01 mm, raw = (P1 + 0.30 mm) x 100 (the upper soft limit)"),
    ParamDef(2, 'Auto-switch "LS" signal detection range adjustment', 8, 9, "mm", "mm", encoding="s32 0.01 mm"),
    ParamDef(3, "HOME direction change", 9, 16, "parity", choices={1: "1:Forward", 0: "0:Reverse"},
             encoding="ORG home pattern number (item 9.16): parity = direction (odd = 1:Forward, even = 0:Reverse); "
                      "a change writes ORG -1 / +1; not settable when ORG is 0 or FFFFFFFF",
             evidence="VERIFIED on the gripper (IA-OS's 1:Forward <-> 0:Reverse changed ORG 7 <-> 8); the parity "
                      "rule for other patterns is UNVERIFIED (the EC-R6S rod holds 7)"),
    ParamDef(4, "HOME position adjustment", 9, 21, "mm", "mm", encoding="s32 0.01 mm"),
    ParamDef(5, "Smooth accel/decel setting", 8, 14, "bit", bit=6, choices=_ENABLE, encoding="bit 6: 1 = Enable"),
    ParamDef(6, "Current control setting while stop", 8, 14, "bit", bit=10, choices=_ENABLE,
             encoding="bit 10: 1 = Enable"),
    ParamDef(7, "Wireless function setting", 8, 50, "bit", bit=23, choices={0: "0:Enable", 1: "1:Disable"},
             encoding="bit 23, inverted sense: 0 = Enable, 1 = Disable"),
    ParamDef(8, "Power-saving setting", 7, 7, "bit", bit=18, choices={0: "0:Enable", 1: "1:Disable"},
             encoding="ATYP item 7.7 bit 18 BUEN: 0 = Enable, 1 = Disable", read_only=True,
             evidence="UNVERIFIED location; fits the EC-R6S rod (ATYP 0x05350000, IA-OS showed 1:Disable)",
             note="read-only until hardware-verified: a P8 change is expected to also rewrite the STN1 (7.15) tuning "
                  "data, set 8.50 b15 INIT and reset CLD0 / CLD1 to FFFFFFFF (speeds / accelerations re-initialised)"),
    ParamDef(9, "Solenoid operation type selection", 8, 2, "enum",
             choices={0: "0:Double", 3: "3:Single", 4: "4:3-position", 5: "5:Double (Multi I/O)"},
             encoding="enum = the number IA-OS shows (valid set by extensibility level, then <= MXPT 3.16)"),
    ParamDef(10, "LED lighting method auto SW setting", 8, 50, "bit", bit=18, choices=_ENABLE,
             encoding="bit 18: 1 = Enable"),
    ParamDef(11, "Multi I/O function selection", 8, 25, "multi_io", item_id2=29,
             choices={0: "0:Not exist", 1: "1:Homing", 2: "2:Pause", 3: "3:Zone 1", 4: "4:Miss-pressing",
                      5: "5:Minor fault", 6: "6:AUTO/MANU switching", 7: "7:Move middle point"},
             encoding="byte 1 (bits 8-15) of FI00 8.25 and FO00 8.29 as a pair", read_only=True,
             evidence=_UNVERIFIED_ITEM, note=_READ_ONLY_UNVERIFIED),
    ParamDef(12, "Move command type", 8, 1, "bit", bit=16,
             choices={0: "0:Continuous energization", 1: "1:Instantaneous energization"},
             encoding="bit 16: 1 = instantaneous (momentary)"),
    ParamDef(13, "Push back error", 9, 15, "bit", bit=10, choices=_ENABLE, encoding="FSTP item 9.15 bit 10 PSER",
             read_only=True, evidence=_UNVERIFIED_ITEM, note=_READ_ONLY_UNVERIFIED),
    ParamDef(14, "Zone output position (1) +", 0x0C, 1, "mm", "mm", encoding="ZNM 0xC.1 entry 0, s32 0.01 mm",
             read_only=True, evidence=_UNVERIFIED_ITEM, note=_READ_ONLY_UNVERIFIED),
    ParamDef(15, "Zone output position (1) -", 0x0C, 2, "mm", "mm", encoding="ZNL 0xC.2 entry 0, s32 0.01 mm",
             read_only=True, evidence=_UNVERIFIED_ITEM, note=_READ_ONLY_UNVERIFIED),
    ParamDef(16, "ACR operation ready signal selection", 8, 50, "bit", bit=10, choices=_ENABLE,
             encoding="SPIO item 8.50 bit 10 ORSS", read_only=True, evidence=_UNVERIFIED_ITEM,
             note=_READ_ONLY_UNVERIFIED),
    ParamDef(17, "Zone output position (2) +", 0x0C, 1, "mm", "mm", encoding="ZNM 0xC.1 entry 1, s32 0.01 mm",
             read_only=True, entry=1, evidence=_UNVERIFIED_ITEM, note=_READ_ONLY_UNVERIFIED),
    ParamDef(18, "Zone output position (2) -", 0x0C, 2, "mm", "mm", encoding="ZNL 0xC.2 entry 1, s32 0.01 mm",
             read_only=True, entry=1, evidence=_UNVERIFIED_ITEM, note=_READ_ONLY_UNVERIFIED),
)  # fmt: skip

EC_CATEGORY_TEXT: dict[str, str] = {
    "A": "customer-adjustable: check or change for the use",
    "B": "use as shipped, as a rule",
    "C": "factory setting for the EC's specification: normally not changed",
}
"""ME3816-6B p.6-6 parameter categories."""

# The public parameter list (DOC ME3816-6B p.6-6; details p.6-7 to 6-17): category, manual name, input range,
# default at shipping, pages. Only P1-P10 are in it; P11-P18 are not (iaos_only).
EC_MANUAL_PARAMETERS: dict[int, dict[str, str]] = {
    1: {"category": "A", "manual_name": "Operation range adjustment",
        "manual_range": "0 to 9999.69 mm (deg); the controller adds 0.30 (p.6-7); above the maximum stroke the "
                        "axis runs into the mechanical stopper",
        "shipping_default": "maximum stroke", "doc": "ME3816-6B p.6-6, 6-7"},
    2: {"category": "A", "manual_name": 'Auto switch "LS" signal detection range adjustment',
        "manual_range": "the actuator's minimum to 9999.99 mm (deg); below the encoder resolution LS never lights "
                        "(p.8-17)",
        "shipping_default": "per EC specification", "doc": "ME3816-6B p.6-6, 6-8"},
    3: {"category": "C", "manual_name": "Home return direction change", "manual_range": "Reverse, Forward",
        "shipping_default": "per EC specification", "doc": "ME3816-6B p.6-6, 6-9; Intro-17; ME3800-5E p.1-36"},
    4: {"category": "B", "manual_name": "Home position adjustment",
        "manual_range": "0 to 9999.99 mm (deg); never below the value at shipping (p.6-10)",
        "shipping_default": "per EC specification", "doc": "ME3816-6B p.6-6, 6-10"},
    5: {"category": "A", "manual_name": "Smooth accel/decel setting", "manual_range": "Disabled, Enabled",
        "shipping_default": "Disabled", "doc": "ME3816-6B p.6-6, 6-11, 8-21"},
    6: {"category": "A", "manual_name": "Current control setting at stop",
        "manual_range": "Disabled (powerful stop), Enabled (energy-saving stop)", "shipping_default": "Disabled",
        "doc": "ME3816-6B p.6-6, 6-12"},
    7: {"category": "A", "manual_name": "Wireless function setting", "manual_range": "Disabled, Enabled",
        "shipping_default": "Enabled", "doc": "ME3816-6B p.6-6, 6-13 (shown only with option WL / WL2)"},
    8: {"category": "A", "manual_name": "Energy-saving setting", "manual_range": "Disabled, Enabled",
        "shipping_default": "Disabled", "doc": "ME3816-6B p.6-6, 6-14"},
    9: {"category": "A", "manual_name": "Electromagnetic valve system select",
        "manual_range": "Double, Single (3-position with option MF); Double only on an ACR (RCON-EC) unit",
        "shipping_default": "Double", "doc": "ME3816-6B p.6-6, 6-15, Intro-9; ME3837-1D p.3-1"},
    10: {"category": "A", "manual_name": "LED lighting system automatic switch setting",
         "manual_range": "Disabled, Enabled; may have no effect outside the small models listed on p.6-16",
         "shipping_default": "Disabled", "doc": "ME3816-6B p.6-6, 6-16"},
}  # fmt: skip
_IAOS_ONLY = {"doc": "not in ME3816-6B (the PC tool lists it, the manual does not)"}
USER_PARAMETERS = tuple(
    replace(p, **EC_MANUAL_PARAMETERS[p.no]) if p.no in EC_MANUAL_PARAMETERS else replace(p, iaos_only=True,
                                                                                          **_IAOS_ONLY)
    for p in USER_PARAMETERS
)  # fmt: skip
READ_ONLY_PARAMETERS: frozenset[int] = frozenset(p.no for p in USER_PARAMETERS if p.read_only)

# Items the driver knows something about beyond the user parameters.
ITEM_NOTES: dict[tuple[int, int], str] = {
    (0x0003, 1): "CTCL: system / unit code + firmware (same as FC11)",
    (0x0003, 2): "MAXP: maximum position count (3 = 3-position)",
    (0x0003, 10): "present alarm code (followed ALMC 0x0EE on the gripper)",
    (0x0003, 16): "MXPT: highest allowed P9 value",
    (0x0003, 52): "SRWT: software reset wait, ms",
    (0x0003, 67): "MINI: minimum positioning band (P2 lower limit with CINP + RINP)",
    (0x0004, 4): "CNOW: motor current",
    (0x0004, 5): "MCUR: rated current (IA-OS shows CNOW / MCUR x 100 %)",
    (0x0007, 1): "LIMM: upper soft limit = P1 + 0.30 mm (0.01 mm)",
    (0x0007, 2): "LIML: lower soft limit (0.01 mm)",
    (0x0007, 3): "LEAD: screw lead (0.01 mm)",
    (0x0007, 4): "MAXV: maximum speed (0.01 mm/s; the driver's speed cap)",
    (0x0007, 5): "MAXA: maximum acceleration (0.01 G; the driver's accel cap)",
    (0x0007, 6): "MAXD: maximum deceleration (0.01 G; the driver's decel cap)",
    (0x0007, 7): "ATYP: axis type bits (b18 BUEN = P8, b24 RINP)",
    (0x0007, 8): "MXPH: maximum push speed (0.01 mm/s)",
    (0x0007, 9): "MXPP: maximum push current % (the hard push cap; 0 = no push)",
    (0x0007, 15): "STN1: smart tuning actuator / lead numbers",
    (0x0007, 33): "AKID: actuator kind (bits 0-7)",
    (0x0007, 34): "TCPC: push force conversion factor (0 = no push)",
    (0x0007, 35): "TCOA: push force conversion offset",
    (0x0007, 58): "MNPP: recommended minimum push % (a warning, not a limit)",
    (0x0008, 3): "BRSL: power-up baud code (6 = 38400)",
    (0x0008, 4): "RTIM: min. delay before the EC's transmitter turns on, ms",
    (0x0008, 10): "VCMD: default command speed (0x0812; not the maximum, which is MAXV 7.4)",
    (0x0008, 11): "ACMD: default acceleration (0.01 G)",
    (0x0008, 15): "PSHV: push speed (0.01 mm/s, capped by MXPH)",
    (0x0008, 54): "FMNT: servo monitor mode (b0-3, 0 = off) / buffering cycle (b16-31)",
    (0x0009, 16): "ORG: home pattern number (P3 = parity); 0x45 home op1",
    (0x000C, 1): "zone boundary ZNM (0.01 mm)",
    (0x000C, 2): "zone boundary ZNL (0.01 mm)",
}


def _param_key(p: ParamDef) -> str:
    cls = f"{p.class_id:X}" if p.class_id > 9 else str(p.class_id)
    return f"{cls}.{p.item_id}" + (f"[{p.entry}]" if p.entry else "") + (f"+{cls}.{p.item_id2}" if p.item_id2 else "")


def home_pattern_settable(raw: int | None) -> bool:
    """P3 can be changed only when the ORG pattern (item 9.16) is neither 0 nor FFFFFFFF (UNVERIFIED)."""
    return raw is not None and raw not in (0, 0xFFFFFFFF)


def _decode_param(p: ParamDef, item: ClassItem | None, item2: ClassItem | None = None) -> dict[str, Any]:
    out: dict[str, Any] = {
        "no": p.no,
        "name": p.name,
        "key": _param_key(p),
        "unit": p.unit,
        "encoding": p.encoding,
        "evidence": p.evidence or _PARAM_EVIDENCE,
        "read_only": p.read_only,
        "category": p.category,
        "category_text": EC_CATEGORY_TEXT.get(p.category, ""),
        "manual_name": p.manual_name,
        "manual_range": p.manual_range,
        "shipping_default": p.shipping_default,
        "doc": p.doc,
        "iaos_only": p.iaos_only,
    }
    if p.note:
        out["note"] = p.note
    if item is None or (p.kind == "multi_io" and item2 is None):
        return {**out, "found": False, "address": None, "raw_hex": None, "value": None, "text": "not found on this EC"}
    out.update(found=True, address=f"0x{item.address:04X}", raw_hex=item.hex)
    value: Any
    if p.kind == "mm":
        value = round(item.signed * R.POSITION_SCALE + p.offset_mm, 2)
        text = f"{value:.2f} mm"
    elif p.kind == "bit":
        value = item.raw >> (p.bit or 0) & 1
        text = (p.choices or {}).get(value, str(value))
    elif p.kind == "parity":
        out["pattern"] = item.raw
        if not home_pattern_settable(item.raw):
            value, text = None, f"not settable (home pattern ORG = 0x{item.raw:08X})"
        else:
            value = item.raw & 1
            text = (p.choices or {}).get(value, str(value))
    elif p.kind == "multi_io":
        assert item2 is not None
        pair = ((item.raw >> 8) & 0xFF, (item2.raw >> 8) & 0xFF)
        out.update(raw_hex=f"{item.hex}/{item2.hex}", address=f"0x{item.address:04X}/0x{item2.address:04X}")
        value = MULTI_IO_PAIRS.get(pair)
        text = (p.choices or {}).get(value, "") if value is not None else f"unknown pair {pair}"
    else:
        value = item.raw
        text = (p.choices or {}).get(item.raw, f"{item.raw}: unknown")
    return {**out, "value": value, "text": text}


def decode_user_parameters(classes: Mapping[int, ClassBlock]) -> list[dict[str, Any]]:
    """The named user parameters P1-P18 (No., name, value, unit, text such as ``"1:Forward"``) from read classes."""
    result = []
    for p in USER_PARAMETERS:
        block = classes.get(p.class_id)
        item = block.item(p.item_id, p.entry) if block is not None else None
        item2 = block.item(p.item_id2, p.entry) if block is not None and p.item_id2 is not None else None
        result.append(_decode_param(p, item, item2))
    return result


def decode_user_parameter(p: ParamDef, item: ClassItem | None, item2: ClassItem | None = None) -> dict[str, Any]:
    """One named user parameter decoded from its class item (same dict as :func:`decode_user_parameters`)."""
    return _decode_param(p, item, item2)


# =========================================================================== user parameter writes (pure)

USER_PARAMETERS_BY_NO: dict[int, ParamDef] = {p.no: p for p in USER_PARAMETERS}
HOME_PARAMETERS: frozenset[int] = frozenset({3, 4})  # home direction / home offset (ABS error 0x0EE after a reset)
PARAMETER_CLASSES_NAMED: tuple[int, ...] = tuple(sorted({p.class_id for p in USER_PARAMETERS}))  # 7, 8, 9, 0x0C
VISIBILITY_CLASSES: tuple[int, ...] = (0x0003, *PARAMETER_CLASSES_NAMED)  # + class 3 (MXPT, MINI) for the rules

PARAM_RAM_ONLY_NOTE = (
    "RAM ONLY - NOT SAVED (commit=False): a parameter written over Modbus (FC10) changes the EC's RAM only and "
    "reverts to the committed value at the next power-up (VERIFIED: P2 0.20 mm went back to 0.10 mm after a "
    "24 V power cycle); probably also at a software reset (untested). A later commit (PARW) saves it too."
)
PARAM_COMMIT_NOTE = (
    "Committed to non-volatile memory with coil 0x0428 (DRGE b7 PARW), the coil IA-OS's Transfer writes (ON-WIRE: "
    "USB capture of IA-OS). PARW saves every parameter value in RAM, including earlier uncommitted writes. The new "
    "value is in the register at once, but the EC applies it only after a software reset or a power cycle "
    "(IA-OS also asks for a software reset after a parameter change)."
)
PARAM_HOME_WARNING = (
    "P3 / P4 are home parameters. After a committed P4 change + software reset the EC-GRBP8M gripper (WA, "
    "battery-less absolute) came up with alarm 0x0EE 'ABS error (2)' (home lost); only an alarm reset + home return "
    "cleared it. Expect the same for P3 / P4 on any absolute (WA) unit; an incremental unit loses its home at every "
    "software reset anyway. Either way: alarm reset (if any) + home return before the next move."
)
# Parameters IA-OS lists whose register is not located (no read, no write). Empty: P8 (item 7.7 b18) and
# P11 / P13-P18 are located (UNVERIFIED, read-only here); kept for existing callers.
NOT_LOCATED_PARAMETERS: dict[int, dict[str, str]] = {}


MODEL_SPECIFIC_KINDS: frozenset[str] = frozenset({"bit", "parity", "enum", "multi_io"})
"""Parameter kinds whose encoding (bit position inside a shared item, ORG parity, enum value) was mapped on the
gripper and checked on the rod. Every unit's parameters are written the same way: there is no per-model opt-in and
no per-model warning."""

OPT_IN_NOTE = ""  # kept for compatibility; no longer used by the driver

# P9 valid values per extensibility level (UNVERIFIED); level >= 2 with EXFN = 0 -> {0, 3}.
_P9_VALID: dict[int, tuple[int, ...]] = {0: (0, 3), 1: (0, 3, 4), 2: (0, 3, 4, 5), 3: (0, 3, 4), 4: (0, 3, 4, 5)}


def extensibility_level(accl: int | None) -> int | None:
    """ACCL (0x0612) bits 8-11: 0 Normal, 1 Position3, 2 Phase1, 3 Phase2, 4 MultiIo (UNVERIFIED)."""
    return None if accl is None else (accl >> 8) & 0xF


def p9_valid_values(level: int | None, exfn: bool | None, mxpt: int | None) -> tuple[int, ...] | None:
    """The valid P9 values on this unit (level / EXFN / MXPT; ``None`` when the level is unknown; UNVERIFIED)."""
    if level is None:
        return None
    valid = _P9_VALID.get(level, (0, 3))
    if level >= 2 and exfn is False:
        valid = (0, 3)
    if mxpt is not None:
        valid = tuple(v for v in valid if v <= mxpt)
    return valid


def parameter_visibility(
    classes: Mapping[int, ClassBlock], support: Mapping[str, int] | None, accl: int | None
) -> dict[int, dict[str, Any]]:
    """Which parameters IA-OS's parameter screen shows on this unit, per parameter number (no I/O).

    Returns ``{no: {"shown": True | False | None, "reason": str}}`` (``None`` = an input is not readable;
    the parameter is then kept). Inputs: ACCL extensibility level (0x0612), EXFN (8.50 b13), SUP2 CNWL (b23,
    P7), SUP3 SORS (b5, P16; no SUP3 = 0), STN1 (7.15; P8, an approximation: shown when STN1 is not
    FFFFFFFF), the current P9 (8.2) against the valid set and MXPT (3.16), FI00 / FO00 (8.25 / 8.29, P11).
    These rules reproduce IA-OS's screen on the gripper (P1-P7, P9, P10, P12) and the rod (P1-P6, P8, P9,
    P12): VERIFIED there, UNVERIFIED on other units. P3 stays shown; it is not settable when ORG (9.16) is
    0 / FFFFFFFF.
    """

    def item(ci: int, item_id: int) -> ClassItem | None:
        block = classes.get(ci)
        return None if block is None else block.item(item_id)

    level = extensibility_level(accl)
    spio = item(R.CATEGORY_MOTION_PARAMS, R.PARAM_ITEM_SPIO)
    exfn = None if spio is None else bool(spio.raw & R.SPIO_EXFN)
    sup2 = None if support is None else support.get("SUP2")
    sup3 = None if support is None else support.get("SUP3", 0)
    stn1 = item(R.CATEGORY_LIMIT_PARAMS, R.AXIS_ITEM_STN1)
    p9 = item(R.CATEGORY_MOTION_PARAMS, 2)
    mxpt = item(R.CATEGORY_CONTROLLER_MONITOR, R.CONTROLLER_ITEM_MXPT)
    lvl = "unknown" if level is None else str(level)

    def rule(cond: bool | None, why: str) -> dict[str, Any]:
        return {"shown": cond, "reason": why}

    ext = None if level is None or exfn is None else (level > 1 and exfn)
    out: dict[int, dict[str, Any]] = {no: rule(True, "") for no in USER_PARAMETERS_BY_NO}
    out[2] = rule(None if ext is None else not (level >= 2 and exfn), "hidden when level >= Phase1 and EXFN = 1")  # type: ignore[operator]
    out[7] = rule(None if sup2 is None else bool(sup2 & R.SUP2_CNWL), "shown only with a wireless module (SUP2 CNWL)")
    out[8] = rule(None if stn1 is None else stn1.raw != 0xFFFFFFFF,
                  "shown only with smart-tuning data (STN1 7.15 set; an approximation)")
    valid = p9_valid_values(level, exfn, None if mxpt is None else mxpt.raw)
    out[9] = rule(None if valid is None or p9 is None else p9.raw in valid,
                  f"the present value must be in the valid set {valid} (level {lvl}, <= MXPT)")
    fi, fo = item(R.CATEGORY_MOTION_PARAMS, 25), item(R.CATEGORY_MOTION_PARAMS, 29)
    known_pair = None if fi is None or fo is None else ((fi.raw >> 8) & 0xFF, (fo.raw >> 8) & 0xFF) in MULTI_IO_PAIRS
    p11 = None if ext is None else (ext and level != 3 and bool(known_pair))
    out[11] = rule(p11, f"needs level > Position3, not Phase2, EXFN = 1 and a known FI00/FO00 pair (level {lvl})")
    for no in (13, 14, 15):
        out[no] = rule(ext, f"needs level > Position3 and EXFN = 1 (level {lvl})")
    out[16] = rule(None if support is None else bool((sup3 or 0) & R.SUP3_SORS), "shown only with SUP3 SORS")
    for no in (17, 18):
        out[no] = rule(None if ext is None else (ext and level > 2), f"needs level > Phase1 and EXFN = 1 (level {lvl})")  # type: ignore[operator]
    return out


def model_parameter_list(
    unit_code: int | None,
    model_name: str | None = None,
    *,
    system_code: int | None = None,
    visibility: Mapping[int, Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    """Which user parameters IA-OS's screen lists for this unit (the driver's per-family candidate lists), no I/O.

    Returns ``{"unit_code", "system_code", "model_name", "family", "known", "candidates", "listed", "hidden",
    "verified", "seen_on", "note"}``:

    * a (system, unit) pair with an IA-OS parameter list recorded (``known`` True): ``candidates`` = that list,
      ``listed`` = the candidates the screen shows after the rules of :func:`parameter_visibility` (when
      ``visibility`` is given; ``hidden`` lists the others with the reason).
    * a unit code without a list here (e.g. a newer EC family), or ``None`` (not readable): ``known`` False,
      ``listed`` every parameter the driver maps.

    Every unit's parameters are written the same way, whatever its model. ``verified`` is kept for compatibility and
    equals ``known``; ``seen_on`` is kept for compatibility, no longer used by the driver (always empty).
    """
    system = 0xEC if system_code is None else system_code
    candidates = R.IAOS_PARAMETER_CANDIDATES.get((system, unit_code)) if unit_code is not None else None
    base = {"unit_code": unit_code, "system_code": system_code, "model_name": model_name,
            "family": R.ec_family(system, unit_code), "seen_on": []}  # fmt: skip
    if candidates is not None:
        assert unit_code is not None
        hidden = [{"no": no, "reason": visibility[no].get("reason", "")} for no in candidates
                  if visibility is not None and no in visibility and visibility[no].get("shown") is False]  # fmt: skip
        hidden_nos = {h["no"] for h in hidden}
        listed = [no for no in candidates if no not in hidden_nos]
        note = f"the parameters IA-OS lists for unit 0x{unit_code:02X} ({base['family'] or '?'})"
        return {**base, "known": True, "candidates": list(candidates), "listed": listed, "hidden": hidden,
                "verified": True, "note": note}  # fmt: skip
    if unit_code is None:
        note = "unit code not readable (FC11 / CTCL): every mapped item found on the unit is shown"
    else:
        note = f"unit code 0x{unit_code:02X}: every mapped item found on the unit is shown"
    return {**base, "known": False, "candidates": [], "listed": sorted(USER_PARAMETERS_BY_NO), "hidden": [],
            "verified": False, "note": note}  # fmt: skip


def needs_model_opt_in(p: ParamDef, model: Mapping[str, Any]) -> bool:
    """Always False: the driver has no per-model opt-in (every unit's parameters are written the same way). Kept for
    compatibility."""
    return False


def annotate_parameters(
    named: Sequence[Mapping[str, Any]],
    unit_code: int | None,
    model_name: str | None = None,
    *,
    system_code: int | None = None,
    visibility: Mapping[int, Mapping[str, Any]] | None = None,
) -> list[dict[str, Any]]:
    """The decoded named parameters with per-model flags added (no I/O).

    Adds ``listed`` (IA-OS's screen lists it for this unit, :func:`model_parameter_list`; for an unknown unit:
    True if found), ``verified_for_model``, ``home_parameter`` (P3 / P4), ``writable`` (found, mapped, not read-only
    and settable; an unlisted parameter is writable too and its write logs a warning),
    ``needs_model_opt_in`` (always False, kept for compatibility),
    ``choices`` (number as IA-OS shows it -> label, ``{}`` for mm), ``shown_by_iaos`` / ``hidden_reason`` (the
    :func:`parameter_visibility` rule) and ``model_note``.
    """
    model = model_parameter_list(unit_code, model_name, system_code=system_code, visibility=visibility)
    out = []
    for prm in named:
        q = dict(prm)
        p = USER_PARAMETERS_BY_NO.get(q["no"])
        listed = q["no"] in model["listed"] and (model["known"] or bool(q.get("found")))
        opt_in = p is not None and needs_model_opt_in(p, model)
        settable = not (p is not None and p.kind == "parity" and q.get("found") and q.get("value") is None)
        read_only = bool(p is not None and p.read_only)
        vis = (visibility or {}).get(q["no"], {})
        q.update(
            listed=listed,
            verified_for_model=model["verified"],
            home_parameter=q["no"] in HOME_PARAMETERS,
            writable=bool(q.get("found")) and p is not None and not read_only and settable,
            needs_model_opt_in=opt_in and not read_only,
            choices={str(k): v for k, v in parameter_choices(p).items()} if p is not None else {},
            read_only=read_only,
            shown_by_iaos=vis.get("shown"),
            hidden_reason=vis.get("reason", "") if vis.get("shown") is False else "",
            model_note="",
        )
        if model["known"] and not listed:
            why = f" ({q['hidden_reason']})" if q["hidden_reason"] else ""
            q["model_note"] = f"not listed by IA-OS for unit 0x{model['unit_code']:02X}{why}"
        elif read_only:
            q["model_note"] = (p.note if p is not None and p.note else "read-only in this driver")
        elif not settable:
            q["model_note"] = "not settable: home pattern ORG (item 9.16) is 0 / FFFFFFFF"
        out.append(q)
    return out


def parameter_choices(p: ParamDef) -> dict[int, str]:
    """Number -> label as IA-OS shows them (e.g. ``{1: "1:Forward", 0: "0:Reverse"}``) of a choice parameter; ``{}``
    for mm."""
    if p.kind == "mm" or not p.choices:
        return {}
    return dict(p.choices)


def parse_parameter_value(p: ParamDef, value: Any) -> float | int:
    """A value for parameter ``p`` (as IA-OS shows it) as a number. Raises ValueError for anything else.

    mm parameters: a finite number (or numeric string), rounded to 0.01 mm. Choice parameters: the
    number (``1``, ``"1"``) or the label as IA-OS shows it, case-insensitive (``"1:Forward"``,
    ``"forward"``, ``"Disable"``; a unique prefix such as ``"instant"`` is accepted). Booleans are
    refused: P7 uses 1 = Disable, so ``True`` would be ambiguous.
    """
    if isinstance(value, bool):
        raise ValueError(f"P{p.no}: pass the number or label IA-OS shows, not {value!r} (P7 uses 1 = Disable)")
    if p.kind == "mm":
        number: Any = value
        if isinstance(value, str):
            try:
                number = float(value.strip())
            except ValueError:
                raise ValueError(f"P{p.no} needs a number in mm, got {value!r}") from None
        if not isinstance(number, (int, float)) or not math.isfinite(number):
            raise ValueError(f"P{p.no} needs a finite number in mm, got {value!r}")
        return round(float(number), 2)
    choices = parameter_choices(p)
    allowed = ", ".join(choices.values())
    if isinstance(value, str):
        text = value.strip().lower()
        if text.lstrip("+-").isdigit():
            number = int(text)
        else:
            names = {n: label.split(":", 1)[-1].strip().lower() for n, label in choices.items()}
            exact = [n for n, label in choices.items() if text in (label.lower(), names[n])]
            prefix = [n for n, name in names.items() if text and name.startswith(text)]
            found = exact or prefix
            if len(found) != 1:
                raise ValueError(f"P{p.no} ({p.name}): {value!r} is not one of {allowed}")
            number = found[0]
    elif isinstance(value, float) and value.is_integer():
        number = int(value)
    elif isinstance(value, int):
        number = value
    else:
        raise ValueError(f"P{p.no} ({p.name}): {value!r} is not one of {allowed}")
    if number not in choices:
        raise ValueError(f"P{p.no} ({p.name}): {value!r} is not one of {allowed}")
    return number


def encode_user_parameter(p: ParamDef, raw: int, number: float | int) -> int:
    """The new 32-bit item for P``p.no`` = ``number`` (from :func:`parse_parameter_value`), every other bit of
    ``raw`` kept (read-modify-write: P5/P6 share item 8.14, P7/P10 8.50, P12 is one bit of 8.1).

    Encodings (VERIFIED on the gripper; for P3 see its ``evidence``): mm items s32 0.01 mm, P1 = upper soft
    limit - 0.30 mm (raw = (P1 + 0.30) x 100); bits 1 / 0, P7 inverted in meaning (1 = Disable, the number IA-OS
    shows is the bit); P3 = the PARITY of the ORG home pattern (odd = 1:Forward): an even
    pattern becomes pattern - 1 for 1:Forward, an odd one pattern + 1 for 0:Reverse, an already matching one is
    kept (raises ValueError when ORG is 0 / FFFFFFFF: not settable); P9 the whole item. Read-only kinds raise.
    """
    raw &= 0xFFFFFFFF
    if p.read_only or p.kind == "multi_io":
        raise ValueError(f"P{p.no} ({p.name}) is read-only in this driver")
    if p.kind == "mm":
        counts = R.to_counts(float(number) - p.offset_mm)
        if not -(1 << 31) <= counts < (1 << 31):
            raise ValueError(f"P{p.no}: {number} mm does not fit a 32-bit item")
        return counts & 0xFFFFFFFF
    if number not in parameter_choices(p):
        raise ValueError(f"P{p.no}: {number!r} is not one of its numbers ({', '.join(parameter_choices(p).values())})")
    if p.kind == "bit":
        mask = 1 << (p.bit or 0)
        return (raw & ~mask & 0xFFFFFFFF) | (mask if number else 0)
    if p.kind == "parity":
        if not home_pattern_settable(raw):
            raise ValueError(f"P{p.no}: the home pattern ORG is 0x{raw:08X}: the home direction is not settable")
        if raw & 1 == int(number):
            return raw  # already the requested direction
        return raw - 1 if int(number) == 1 else raw + 1
    return int(number) & 0xFFFFFFFF  # enum: the whole item


# =========================================================================== EC parameter guards (pure)


class ParameterRefused(ValueError):
    """A parameter write the driver refuses (nothing written). ``opt_in`` names the guard token that allows it
    anyway (passed in ``allow`` to :func:`ec_parameter_guard`), ``""`` when no opt-in exists."""

    def __init__(self, message: str, opt_in: str = "") -> None:
        super().__init__(message)
        self.opt_in = opt_in


SMOOTH_ACCEL_LIMIT_S = 2.0
"""P5 (smooth accel/decel) must stay disabled when any acceleration or deceleration takes longer than this (DOC
ME3816-6B p.6-11)."""
_G_MM_S2 = 9806.65


def find_user_parameter(key: Any) -> ParamDef | None:
    """The EC user parameter for ``key``: its number (``5``, ``"5"``, ``"P5"``), the driver's name or the manual's
    name (:attr:`ParamDef.manual_name`, e.g. ``"Energy-saving setting"``; case-insensitive), else ``None``."""
    if isinstance(key, bool):
        return None
    if isinstance(key, int):
        return USER_PARAMETERS_BY_NO.get(key)
    text = str(key).strip()
    number = text[1:] if text[:1] in ("P", "p") else text
    if number.isdigit():
        return USER_PARAMETERS_BY_NO.get(int(number))
    low = text.lower()
    return next((p for p in USER_PARAMETERS if low in (p.name.lower(), p.manual_name.lower())), None)


def has_acr_option(model_name: str | None) -> bool | None:
    """Whether the model string carries option ACR (RCON-EC connection, ME3816-6B p.1-9) as an option code: a
    hyphen-separated token after the type and stroke (``EC-S6SAH-100-...-ACR``); the ``ACR`` inside a clean-room
    type name such as ``S6SACR`` does not count. ``None`` when the model string is not known."""
    if not model_name:
        return None
    tokens = [t.strip().upper() for t in str(model_name).split("-")]
    return "ACR" in tokens[2:]


def ramp_time_s(speed: float, accel_g: float) -> float:
    """Time to reach ``speed`` (units/s) at ``accel_g`` (G = 9806.65 mm/s^2); ``inf`` for a zero acceleration."""
    return math.inf if accel_g <= 0 else abs(speed) / (accel_g * _G_MM_S2)


def smooth_accel_check(
    speed: float, accel_g: float, decel_g: float | None = None, *, smooth_enabled: bool | None, retarget: bool = False
) -> dict[str, Any]:
    """The P5 (smooth accel/decel) rules for one move (no I/O; DOC ME3816-6B p.6-11, p.8-21 cause 7).

    Returns ``accel_time_s`` / ``decel_time_s``, ``over_limit`` (P5 enabled and a ramp longer than
    :data:`SMOOTH_ACCEL_LIMIT_S`: "normal operation will not be possible") and ``warnings``. With P5 enabled a
    ``retarget`` (a new command during a move, e.g. a speed change) warns that a command issued too soon can raise an
    overload alarm (group A). ``smooth_enabled`` ``None`` (P5 not read) or False gives no warning."""
    decel = accel_g if decel_g is None else decel_g
    up, down = ramp_time_s(speed, accel_g), ramp_time_s(speed, decel)
    warnings: list[str] = []
    over = bool(smooth_enabled) and max(up, down) > SMOOTH_ACCEL_LIMIT_S
    if over:
        warnings.append(f"P5 smooth accel/decel is enabled and this move ramps for {max(up, down):.2f} s (> "
                        f"{SMOOTH_ACCEL_LIMIT_S:g} s): normal operation is not possible (ME3816-6B p.6-11); raise "
                        "the acceleration / deceleration or disable P5")  # fmt: skip
    if smooth_enabled and retarget:
        warnings.append("P5 smooth accel/decel is enabled: a new command issued too soon during a move can raise an "
                        "overload alarm (group A; ME3816-6B p.8-21 cause 7); do not pause during a ramp "
                        "(ME3816-6B p.6-11)")  # fmt: skip
    return {"accel_time_s": round(up, 3), "decel_time_s": round(down, 3), "over_limit": over, "warnings": warnings}


def ec_parameter_guard(
    p: ParamDef,
    number: float | int,
    *,
    current: float | int | None = None,
    shipped: float | int | None = None,
    rotary: bool = False,
    model_name: str | None = None,
    allow: Iterable[str] = (),
) -> list[str]:
    """The manual's rules for writing EC user parameter ``p`` = ``number`` (no I/O; ME3816-6B). Returns the
    warnings to report; raises :class:`ParameterRefused` (nothing written) when a rule refuses, naming the
    ``allow`` token that would let it through.

    * P4 (home position adjustment) below the value at shipping is refused (p.6-10: "do not set ... smaller"; IAI
      asks to be contacted): the floor is ``shipped`` (e.g. from a backup, :func:`backup`) or, when that is not
      known, the ``current`` value. Token ``"p4_below_shipped"``. Every P4 write warns to adjust P1 with it.
    * P3 (home direction) on a rotary axis is refused (``rotary``; a manual rule; changing P3 alone does
      not reverse the home of a rotary: the stopper must be swapped, ME3800-5E p.1-36; some models cannot reverse at
      all, ME3816-6B Intro-17). Token ``"rotary_home_direction"``. Every P3 write warns that every move reverses.
    * P9 = 3 (Single) on a unit with option ACR (:func:`has_acr_option`) is refused (p.6-15, Intro-9: an ACR unit on
      RCON-EC must stay Double). Token ``"acr_single"`` (e.g. an ACR unit used on its own SIO port).
    * P5 enabled: warns about the 2 s ramp rule and re-targeting (p.6-11, p.8-21); P10: may have no effect outside
      the small models (p.6-16); P11-P18: not in ME3816-6B.
    """
    allowed = set(allow)
    warnings: list[str] = []
    if p.no == 4:
        floor = shipped if shipped is not None else current
        what = ("the value at shipping" if shipped is not None
                else "the present value (the value at shipping is not known)")
        if floor is not None and number < float(floor) - 1e-6 and "p4_below_shipped" not in allowed:
            raise ParameterRefused(
                f"P4 (home position adjustment) = {number:.2f} is below {what} {float(floor):.2f}: ME3816-6B p.6-10 "
                "forbids a value smaller than the default at shipping (abnormal home return, alarms); IAI asks to be "
                "contacted first", "p4_below_shipped")  # fmt: skip
        warnings.append("P4: adjust P1 (operation range) together with P4 (P1's datum is the home position) and do "
                        "a home return (absolute reset) after the change (ME3816-6B p.6-10)")  # fmt: skip
    if p.no == 3:
        if rotary and "rotary_home_direction" not in allowed:
            raise ParameterRefused(
                "P3 (home direction) on a rotary axis: changing P3 alone does not reverse the home position (the "
                "stopper must be swapped, ME3800-5E p.1-36; some models cannot reverse at all, ME3816-6B Intro-17)",
                "rotary_home_direction")  # fmt: skip
        warnings.append("P3: the direction of every move reverses; a home return (absolute reset) is required "
                        "afterwards (ME3816-6B p.6-9)")  # fmt: skip
    if p.no == 9 and int(number) == 3 and has_acr_option(model_name) and "acr_single" not in allowed:
        raise ParameterRefused(
            "P9 = Single on a unit with option ACR (RCON-EC connection): it must stay Double, or the EC may not "
            "operate as commanded (ME3816-6B p.6-15, Intro-9)", "acr_single")  # fmt: skip
    if p.no == 5 and int(number) == 1:
        warnings.append(f"P5 enabled: no acceleration or deceleration may take longer than {SMOOTH_ACCEL_LIMIT_S:g} s, "
                        "avoid pausing during a ramp (ME3816-6B p.6-11), and a new command issued too soon during a "
                        "move can raise an overload alarm (ME3816-6B p.8-21 cause 7)")  # fmt: skip
    if p.no == 10 and int(number) == 1:
        warnings.append("P10: may have no effect outside the small models listed in ME3816-6B p.6-16")
    if p.iaos_only:
        warnings.append(f"P{p.no} is not in ME3816-6B (IA-OS lists it): its meaning is UNVERIFIED")
    return warnings


def ec_parameter_table() -> list[dict[str, Any]]:
    """The EC user parameters as rows a GUI can show (no I/O): ``no``, ``name``, ``manual_name``, ``category`` /
    ``category_text``, ``unit``, ``manual_range``, ``shipping_default``, ``driver_range`` (the narrower limit the
    driver enforces, ``None`` when it has none), ``choices``, ``read_only``, ``home_parameter``, ``iaos_only`` and
    ``doc``. Plus No.143 (overload warning level), which the manual names but the driver does not map."""
    from iai_actuator_driver import ec as _ec  # absolute: runs re-bound in info.py too

    driver: dict[int, tuple[float | None, float | None]] = {}
    for no, name in ((1, "P1_MIN_MM"), (2, "P2_RANGE_MM"), (4, "P4_RANGE_MM")):
        limit = getattr(_ec, name, None)
        if isinstance(limit, (int, float)):
            driver[no] = (float(limit), None)  # P1: up to the largest P1 read from the unit
        elif isinstance(limit, tuple) and len(limit) == 2:
            driver[no] = (limit[0], limit[1])
    rows: list[dict[str, Any]] = []
    for p in USER_PARAMETERS:
        lo_hi = driver.get(p.no)
        rows.append({
            "no": p.no, "name": p.name, "manual_name": p.manual_name, "category": p.category,
            "category_text": EC_CATEGORY_TEXT.get(p.category, ""), "unit": p.unit, "manual_range": p.manual_range,
            "shipping_default": p.shipping_default,
            "driver_range": None if lo_hi is None else {"min": lo_hi[0], "max": lo_hi[1]},
            "choices": {str(k): v for k, v in parameter_choices(p).items()}, "read_only": p.read_only,
            "home_parameter": p.no in HOME_PARAMETERS, "iaos_only": p.iaos_only, "doc": p.doc,
        })  # fmt: skip
    rows.append({
        "no": 143, "name": "Overload warning level", "manual_name": "Overload Warning Load Level Ratio",
        "category": "", "category_text": "", "unit": "%", "manual_range": "50-99 %, 100 = off",
        "shipping_default": "100", "driver_range": None, "choices": {}, "read_only": True, "home_parameter": False,
        "iaos_only": False, "doc": "ME3816-6B p.5-3 (maintenance information; register not mapped by the driver)",
    })  # fmt: skip
    return rows


def annotate_con_parameters(numbered: Sequence[Mapping[str, Any]], family: str | None) -> list[dict[str, Any]]:
    """The numbered CON parameters (``Elecylinder.parameters()`` rows) with the family's documented view added (no
    I/O): ``family_min`` / ``family_max`` (:func:`iai_actuator_driver.con_params.for_family`), ``category`` /
    ``category_text``, ``factory_default``, ``range_doc``, ``dangerous`` (also every D / E category of the family)
    and ``note``."""
    fam = con_params.con_family(family)
    out = []
    for row in numbered:
        q = dict(row)
        p = con_params.CON_PARAMETERS_BY_NO.get(q.get("number", -1))
        if p is not None:
            f = con_params.for_family(p, fam)
            cat = con_params.category(p.no, fam)
            lo, hi = f.lo, f.hi
            if p.kind == "baud":
                lo, hi = min(con_params.CON_BAUD_RATES), max(con_params.CON_BAUD_RATES)
            q.update(
                family_min=lo, family_max=hi, category=cat, category_text=con_params.CATEGORY_TEXT.get(cat, ""),
                factory_default=con_params.factory_default(p.no, fam),
                range_doc=con_params.FAMILY_RANGE_DOC.get(fam or "PCON", ""),
                dangerous=bool(q.get("dangerous")) or f.dangerous, note=p.note,
            )  # fmt: skip
        out.append(q)
    return out


def con_range_check(
    numbered: Sequence[Mapping[str, Any]], family: str | None, limits: con_params.ConUnitLimits
) -> dict[str, Any]:
    """Audit what a CON unit holds (:func:`iai_actuator_driver.con_params.audit`; no I/O): ``family`` (``None`` =
    the family has no range table of its own; the PCON-CB range is used), ``unit_limits``
    (:class:`iai_actuator_driver.con_params.ConUnitLimits`), ``min_band`` (No.10 floor,
    :func:`iai_actuator_driver.con_params.min_positioning_band`) and ``findings`` (values a write would refuse; a No.10
    below the minimum band means cold-start alarm 0x0A1 at the next reset)."""
    fam = con_params.con_family(family)
    values = {int(row["number"]): row.get("value") for row in numbered if "number" in row}
    band = con_params.min_positioning_band(fam, limits.lead_mm, limits.encoder_pulses, mini_mm=limits.min_band_mm)
    return {
        "family": fam,
        "unit_limits": limits.to_dict(),
        "min_band": None if band is None else {"mm": band.mm, "formula": band.formula, "doc": band.doc,
                                               "certain": band.certain},
        "findings": con_params.audit(values, family=fam, unit=limits),
        "note": "" if fam is not None else con_params.UNKNOWN_FAMILY_NOTE,
    }


# =========================================================================== input ranges per product line

PRODUCT_LINES: tuple[str, ...] = ("EC", "CON-CB", "RCON", "ROBONET")

_MOVE_FIELDS: list[dict[str, str]] = [
    {"field": "Target (PCMD)", "register": "0x9900 s32", "raw_range": "-999,999 .. 999,999",
     "range": "+/-9,999.99 mm (deg)",
     "limit": "the soft limits minus the 0.30 margin; outside is refused (the unit would clamp to 0.2 mm "
                     "inside its soft limit)", "doc": "ME0162-12A p.5-131"},
    {"field": "Band (INP)", "register": "0x9902 u32", "raw_range": "1 .. 999,999 (not 0)", "range": "0.01 .. 9,999.99",
     "limit": ">= MINI 3.67 when enforced; CON: >= lead / pulses x 3 on low-resolution steppers; push band = "
                     "push distance, at most the distance to the stroke end",
     "doc": "ME0162-12A p.5-131, 5-151; ME0342-4H p.6-12"},
    {"field": "Speed (VCMD)", "register": "0x9904 u32", "raw_range": "1 .. 999,999", "range": "0.01 .. 9,999.99 mm/s",
     "limit": "<= MAXV (7.4); above the unit's maximum the unit alarms at start; below the documented "
                     "minimum speed: a warning", "doc": "ME0162-12A p.5-131"},
    {"field": "Acceleration (ACMD)", "register": "0x9906", "raw_range": "1 .. 300", "range": "0.01 .. 3.00 G",
     "limit": "<= MAXA / MAXD (7.5 / 7.6); 0.3 G while the unit's maximum is not known",
     "doc": "ME0162-12A p.5-132"},
    {"field": "Push current (PPOW)", "register": "0x9907", "raw_range": "0x33 .. 0xB2 = 20-70 % (RA13R / RA15R / "
     "RA20R: 0x33 .. 0x1FE)", "range": "raw = floor(% x 255 / 100)",
     "limit": "<= MXPP (7.9); below MNPP (7.58, else 20 %): a warning; push speed = min(VCMD, PSHV, MXPH)",
     "doc": "ME0162-12A p.5-132"},
    {"field": "Control flags (CTLF)", "register": "0x9908", "raw_range": "b1 PUSH, b2 DIR (1 = away from home), "
     "b3 INC, b4-5 GSL, b6-7 MOD, b12-13 NTC", "range": "-",
     "limit": "GSL / NTC only on the ACON / SCON models listed in the manual; DIR = 1 pushes away from home "
                     "(VERIFIED)", "doc": "ME0162-12A p.5-134"},
    {"field": "Zones (rows)", "register": "ZNMP / ZNLP s32", "raw_range": "s32", "range": "mm",
     "limit": "any order: Zone+ < Zone- is an inverted zone; equal = off", "doc": "ME0342-4H p.4-31, 6-7"},
    {"field": "Rows", "register": "-", "raw_range": "No.25: 64 / 64 / 256 / 512 / 7 / 3 for patterns 0-5",
     "range": "-", "limit": "from the unit (MAXP 3.2 / the directory)", "doc": "ME0342-4H p.4-9, 6-17"},
]  # fmt: skip

INPUT_RANGES: dict[str, dict[str, Any]] = {
    "EC": {
        "title": "ELECYLINDER (own SIO port)",
        "rule": "the unit's own limits first (soft limits, MAXV, MAXA / MAXD, MXPP, MXPH, MINI), then the manual "
                "range, then a driver default; a documented minimum is a warning, never a refusal",
        "moves": _MOVE_FIELDS,
        "parameters": "ec_parameter_table()",
        "other": [
            {"field": "Operation noise", "range": "0-14 (Ultra Mini 1-9); needs a reboot",
             "doc": "ME3816-6B p.4-16, 4-17"},
            {"field": "Automatic servo OFF delay", "range": "0.1 s steps, 0 = off, one per direction; not used with "
             "pushing (the work is released)", "doc": "ME3816-6B p.4-20, 4-21, 4-52"},
            {"field": "Maintenance targets", "range": "travel count 0-999,999,999, distance 0-999,999,999 m, 0 = off; "
             "overload warning level 50-100 %, 100 = off", "doc": "ME3816-6B p.5-1 to 5-7"},
            {"field": "Homing speed", "range": "the unit's own (20 mm/s in the manual; about 30 mm/s on an EC-DS4H, "
             "VERIFIED)", "doc": "ME3816-6B p.4-33"},
        ],
    },
    "CON-CB": {
        "title": "PCON / ACON / DCON / SCON-CB controllers",
        "rule": "per-family parameter ranges (con_params.parameter_table(family)); the unit's maxima cap No.8 / 26 / "
                "35 / 47 (MAXV), No.9 (MAXA), No.34 (MXPH); No.10 >= the minimum positioning band",
        "moves": _MOVE_FIELDS,
        "parameters": "con_params.parameter_table(family)",
        "other": [
            {"field": "No.16 SIO baud", "range": "9,600 / 14,400 / 19,200 / 28,800 / 38,400 / 76,800 / 115,200 / "
             "230,400; default 38,400", "doc": "ME0342-4H p.6-14; ME0343-3B p.6-14; ME0340-9F p.6-16"},
            {"field": "No.29 excitation detection time", "range": "1-999 ms (ACON 50-999); drives the first servo-on "
             "time", "doc": "ME0342-4H p.6-20; ME0343-3B p.6-3"},
            {"field": "No.30 excitation detection type", "range": "0 / 1 / 2; 2 on a vertical axis may drop the load",
             "doc": "ME0342-4H p.6-20"},
            {"field": "No.42 enable function", "range": "default 1 on p.6-3 but 0 on p.6-25 (manual inconsistency)",
             "doc": "ME0342-4H p.6-3, 6-25"},
            {"field": "No.168 collision detection", "range": "0-7 (PCON only)", "doc": "ME0342-4H p.6-53, 5-3"},
            {"field": "Rotary index mode", "range": "No.79 = 1: 0-359.99 deg, soft limits off, no push",
             "doc": "ME0342-4H p.4-11, 6-35"},
            {"field": "Parameter commit", "range": "non-volatile write + software reset or power cycle; no write limit "
             "on CB units (FeRAM)", "doc": "ME0342-4H p.6-1, 2-11"},
        ],
    },
    "RCON": {
        "title": "RCON driver units behind an RCON-GW",
        "rule": "only the documented path: FC 03 / 06 / 10 / 17 (+ 01 / 04), no broadcast, MANU mode required",
        "moves": _MOVE_FIELDS,
        "parameters": None,
        "other": [
            {"field": "Line", "range": "RTU only, 115,200 bps fixed (a PC tool may leave its own rate)",
             "doc": "ME0413-2B p.2-1, 6-1; ME0384-9B p.2-4"},
            {"field": "Addresses", "range": "gateway D0h (D1h) or FFh; axes 01h-10h; at most 16 axes incl. EC axes",
             "doc": "ME0413-2B p.4-1; ME0384-9B p.2-3, 3-117"},
            {"field": "Function codes", "range": "01, 03, 04, 06, 10, 17 only; no broadcast",
             "doc": "ME0413-2B p.3-1, 6-1"},
            {"field": "Position unit", "range": "No.207: 0.01 or 0.001 mm (target, band, zones, PNOW)",
             "doc": "ME0384-9B p.8-56"},
            {"field": "Push", "range": "255 = 100 %, 510 = 200 % (fieldbus)",
             "doc": "ME0384-9B p.6-32; ME0413-2B p.5-12"},
            {"field": "No.35 safety speed", "range": "1-250 mm/s, default 100", "doc": "ME0384-9B p.8-2"},
            {"field": "Completed position (POSS)", "range": "7 bits (row <= 127)", "doc": "ME0413-2B p.5-10 to 5-23"},
        ],
    },
    "ROBONET": {
        "title": "ROBONET (RGW-SIO gateway)",
        "rule": "SIO through mode (SW2 ON) for direct access; gateway mode is not supported",
        "moves": _MOVE_FIELDS,
        "parameters": None,
        "other": [
            {"field": "Baud", "range": "through mode 38,400-230,400; gateway mode 9,600-230,400",
             "doc": "ME0208-14B p.26"},
            {"field": "Slave / function codes", "range": "through: axis + 1, FC 01-07, 0F, 10, 11, 17; gateway: 3Fh, "
             "FC 03, 06, 10", "doc": "ME0208-14B p.26, 119-123"},
            {"field": "Buffer", "range": "160 bytes (at most 75 registers per FC10)", "doc": "ME0208-14B p.26"},
            {"field": "Gateway-mode units", "range": "speed 1 mm/s (or 0.1), accel = decel <= 2 G (0.01 G), push "
             "0-FFh, 32-bit low word first", "doc": "ME0208-14B p.71-75, 131"},
            {"field": "Internal link", "range": "230.4 kbps fixed; external-SIO SCON / PCON-CF need No.16 = 230,400 "
             "and No.17 = 2 ms", "doc": "ME0208-14B p.340, 348"},
        ],
    },
}  # fmt: skip
"""Input ranges per product line, for a GUI (no I/O; :func:`input_ranges`): ``title``, ``rule``, ``moves`` (the
direct-move and row fields), ``parameters`` (where the parameter table comes from) and ``other`` settings."""


def product_line(family: str | None, gateway: str | None = None) -> str:
    """The :data:`INPUT_RANGES` key for a unit: ``"RCON"`` / ``"ROBONET"`` behind those gateways, ``"EC"`` for an EC,
    ``"CON-CB"`` for every other family (a generic IAI controller included)."""
    if gateway:
        gw = str(gateway).lower()
        if gw.startswith("rcon"):
            return "RCON"
        if gw.startswith("robonet"):
            return "ROBONET"
    return "EC" if str(family or "").upper() == "EC" else "CON-CB"


def input_ranges(line: str | None = None, *, family: str | None = None) -> dict[str, Any]:
    """:data:`INPUT_RANGES` for one product ``line`` (or every line when ``None``) with the parameter rows filled in
    (:func:`ec_parameter_table` for the EC, :func:`iai_actuator_driver.con_params.parameter_table` for ``family`` on CON-CB).
    JSON-friendly; no I/O."""
    lines = PRODUCT_LINES if line is None else (line,)
    out: dict[str, Any] = {}
    for name in lines:
        if name not in INPUT_RANGES:
            raise ValueError(f"unknown product line {name!r}; choose from {list(PRODUCT_LINES)}")
        entry = dict(INPUT_RANGES[name])
        if name == "EC":
            entry["parameters"] = ec_parameter_table()
        elif name == "CON-CB":
            entry["parameters"] = con_params.parameter_table(family)
            entry["family"] = con_params.con_family(family)
        out[name] = entry
    return out if line is None else out[line]


def _annotated_class(block: ClassBlock) -> dict[str, Any]:
    data = block.to_dict()
    params: dict[tuple[int, int], list[str]] = {}
    for p in USER_PARAMETERS:
        if p.class_id == block.class_id:
            for item_id in (p.item_id, p.item_id2):
                if item_id is not None:
                    params.setdefault((item_id, p.entry), []).append(f"P{p.no}")
    for item, raw in zip(data["items"], block.items):
        if raw.item_id is not None:
            item["params"] = params.get((raw.item_id, raw.entry), [])
            item["note"] = ITEM_NOTES.get((block.class_id, raw.item_id), "")
    return data


# =========================================================================== support functions (class 1)

# Bit names of the class-1 words (IAI bit names, meanings UNVERIFIED; values VERIFIED-LIVE on the slider / gripper).
SUPPORT_BITS: dict[str, dict[int, str]] = {
    "SUPT": {0: "STOP", 1: "EMG", 2: "SRST software reset", 3: "ROAX index mode (not 'rotary')", 4: "JOGH",
             5: "AUNT absolute unit", 6: "SFTY safety speed", 7: "PIO", 8: "INCM relative moves",
             9: "ABSM absolute moves", 10: "PULS", 11: "PUSH push motion", 12: "DRAT", 13: "ABSE absolute encoder",
             14: "CPIO", 15: "AXSW", 29: "MBRT Modbus RTU", 30: "MBAS Modbus ASCII", 31: "MBTC Modbus TCP"},
    "MVPT": {0: "TRAP", 1: "SCUR", 2: "PFIL", 4: "PTPM", 5: "VELM", 6: "TRQM", 7: "DDRV", 8: "ASOF", 9: "SMOD",
             10: "VCTL vibration control", 11: "GSLC", 12: "ASOE", 13: "CDAS collision detection"},
    "SPTS": {0: "LS", 1: "CREP", 2: "OT", 3: "HMCK home check sensor", 4: "BLCT", 7: "ENBL enable switch",
             8: "MDSW", 9: "PTSW", 10: "FANE", 11: "SDWN", 12: "PP", 13: "SWJ+", 14: "NP", 15: "SWJ-"},
    "SUP2": {0: "FCTL", 1: "MONI monitoring", 2: "RTC", 3: "OVLW overload warning", 4: "TKTM tact time",
             5: "MACA", 6: "UNIT", 7: "TRG monitoring trigger", 12: "BTRG", 13: "MNSC 0x46", 18: "MABS battery-less ABS",
             19: "MAPI pairing ID", 23: "CNWL wireless module", 24: "CINP min. band limit", 25: "CVEL min. speed limit",
             28: "STO"},
    "SUP3": {0: "VAX virtual axis", 1: "SLUP", 2: "TNF", 3: "DREC", 4: "RALM", 5: "SORS ACR ready signal",
             16: "IVC1 uses 0x48"},
}  # fmt: skip


def decode_support_words(words: Mapping[str, int] | None) -> dict[str, Any] | None:
    """Class-1 support words decoded with :data:`SUPPORT_BITS` plus the driver-relevant flags (no I/O).

    ``flags["absolute_encoder"]``: :func:`iai_actuator_driver.alarms.absolute_encoder_from_support` (SUPT b13 ABSE /
    SUP2 b18 MABS; ``None`` when it cannot be told)."""
    if words is None:
        return None
    out: dict[str, Any] = {"words": {}, "evidence": "IAI bit names (meanings UNVERIFIED); values read from the EC"}
    for name, value in words.items():
        width = 8 if name in ("SUPT", "SUP2", "AEHS", "ACHS", "ARSF", "SPTM", "SUP3") else 4
        names = SUPPORT_BITS.get(name, {})
        out["words"][name] = {
            "value": value, "hex": f"0x{value:0{width}X}",
            "on": [names.get(b, f"b{b}") for b in range(4 * width) if value >> b & 1],
        }  # fmt: skip
    supt, mvpt, sup2, sup3 = (words.get(k) for k in ("SUPT", "MVPT", "SUP2", "SUP3"))
    out["flags"] = {
        "push": None if supt is None else bool(supt & R.SUPT_PUSH),
        "relative_moves": None if supt is None else bool(supt & R.SUPT_INCM),
        "software_reset": None if supt is None else bool(supt & R.SUPT_SRST),
        "collision_detection": None if mvpt is None else bool(mvpt & R.MVPT_CDAS),
        "vibration_control": None if mvpt is None else bool(mvpt & R.MVPT_VCTL),
        "wireless_module": None if sup2 is None else bool(sup2 & R.SUP2_CNWL),
        "absolute_encoder": absolute_encoder_from_support(words),
        "uses_function_0x48": bool(sup3 & R.SUP3_IVC1) if sup3 is not None else False,
        "has_sup3": sup3 is not None,
    }
    return out


# =========================================================================== maintenance

# Item ID -> (symbol, meaning, confirmed). TLMC / ODOM VERIFIED-LIVE (TLMC +1 per move start); the other
# symbols are IAI's maintenance item names in their expected order (UNVERIFIED).
MAINTENANCE_ITEMS: dict[int, tuple[str, str, bool]] = {
    1: ("TLMC", "total moves (move starts, retargets included)", True),
    2: ("ODOM", "total distance (m)", True),
    3: ("FMAX", "", False), 4: ("VMAX", "", False), 5: ("MMAX", "", False), 6: ("CMAX", "", False),
    7: ("TTIM", "", False), 8: ("EMGC", "", False),
    9: ("TEMP", "two 16-bit values (rose by 2 after moves); temperatures?", False),
    10: ("TTPM", "", False), 11: ("ADMX", "", False), 12: ("TARM", "", False), 13: ("TACM", "", False),
    14: ("OLMX", "", False), 15: ("TOLM", "", False), 16: ("TLM2", "", False), 17: ("ODO2", "", False),
    18: ("TIMN", "", False), 19: ("TFMX", "", False), 20: ("TVMX", "", False), 21: ("TMMX", "", False),
    22: ("TCMX", "", False), 23: ("TLM3", "", False), 24: ("ODO3", "", False), 25: ("ACHT", "", False),
    26: ("TFAN", "", False), 27: ("CAOC", "", False), 28: ("CAOH", "", False), 29: ("CAMV", "", False),
}  # fmt: skip


# TIMN (the calendar's current time) per CON family (DOC ME0162-12A p.4-16): SCON-CA/CAL/CB 0x841E, PCON-CA/CFA/CB/
# CFB/CBP 0x8420, ACON / DCON-CA/CB 0x8422. Seconds since 2000-01-01 when the calendar is in use (SSSE b8).
TIMN_ADDRESS: dict[str, int] = {"SCON": 0x841E, "PCON": 0x8420, "ACON/DCON": 0x8422}
SSSE_URTC = 1 << 8  # SSSE b8: calendar (RTC) in use (DOC ME0162-12A p.4-25)


def rtc_in_use(ssse: int | None) -> bool | None:
    """SSSE b8 (the unit's calendar is in use: time stamps are seconds since 2000-01-01), ``None`` when not read."""
    return None if ssse is None else bool(ssse & SSSE_URTC)


def _read_ssse(ec: Elecylinder, monitor: Mapping[str, int]) -> int | None:
    """SSSE from monitor words already read, else one FC03 read of 0x9015 (``None`` if it does not answer)."""
    if "SSSE" in monitor:
        return monitor["SSSE"]
    names = [name for name, _ in _MONITOR_LAYOUT]
    offset = sum(size for _, size in _MONITOR_LAYOUT[: names.index("SSSE")])  # 0x9015
    try:
        return ec.read_registers(R.MONITOR_BASE + offset, 1)[0]
    except (ModbusException, ModbusTimeout):
        return None


def decode_maintenance(block: ClassBlock, *, rtc: bool | None = None, family: str | None = None) -> dict[str, Any]:
    """The maintenance class (12) as :func:`unit_info` reports it: the items with their meaning and, on a CON
    controller whose clock is in use (``rtc``), the TIMN calendar time (``family``: the product family)."""
    timn_address = TIMN_ADDRESS.get(con_params.con_family(family) or "")
    calendar_time: str | None = None
    items = []
    for it in block.items:
        symbol, meaning, confirmed = MAINTENANCE_ITEMS.get(it.item_id or -1, ("?", "", False))
        entry = {
            "id": it.item_id,
            "symbol": symbol,
            "meaning": meaning,
            "confirmed": confirmed,
            "address": f"0x{it.address:04X}",
            "hex": it.hex,
            "value": it.raw,
            "unset": it.raw == 0xFFFFFFFF,
        }
        if symbol == "TEMP":
            entry.update(hi16=it.raw >> 16, lo16=it.raw & 0xFFFF)
        if it.address == timn_address and it.size == 2:
            entry["meaning"] = "TIMN: the calendar's current time (DOC ME0162-12A p.4-16)"
            if rtc and it.raw != 0xFFFFFFFF:
                calendar_time = entry["time"] = rtc_datetime(it.raw).isoformat(sep=" ")
        items.append(entry)
    moves = block.item(R.MAINTENANCE_TOTAL_MOVES_ID)
    dist = block.item(R.MAINTENANCE_DISTANCE_ID)
    return {
        "address": f"0x{block.address:04X}",
        "total_moves": None if moves is None else moves.raw,
        "distance_m": None if dist is None else dist.raw,
        "calendar_time": calendar_time,
        "items": items,
        "note": "TLMC / ODOM VERIFIED-LIVE (total distance in m: DOC ME0162-12A p.4-15); the other symbols are in "
        "their expected order (UNVERIFIED); FFFFFFFF = not set; calendar_time = TIMN as seconds since 2000-01-01 "
        "when the unit's calendar is in use (SSSE b8)",
    }


# =========================================================================== monitor / status words

# 0x9000 monitor members (IAI names and order; offsets VERIFIED-LIVE up to ZONS on both units, OLLV on the slider).
_MONITOR_LAYOUT: tuple[tuple[str, int], ...] = (
    ("PNOW", 2), ("ALMC", 1), ("DIPM", 1), ("DOPM", 1), ("DSS1", 1), ("DSS2", 1), ("DSSE", 1), ("STAT", 2),
    ("VNOW", 2), ("CNOW", 2), ("DEVI", 2), ("STIM", 2), ("SIPM", 1), ("ZONS", 1), ("POSS", 1), ("SSSE", 1),
    ("PNWP", 2), ("VNWP", 2), ("CNWF", 2), ("PDNW", 2), ("FBFC", 2), ("OLLV", 2),
)  # fmt: skip


def _split_monitor(words: Sequence[int]) -> dict[str, int]:
    """Monitor words by name (unsigned; only the members fully inside ``words``)."""
    out: dict[str, int] = {}
    offset = 0
    for name, size in _MONITOR_LAYOUT:
        if offset + size > len(words):
            break
        out[name] = words[offset] if size == 1 else (words[offset] << 16) | words[offset + 1]
        offset += size
    return out


def _s32(value: int) -> int:
    return value - (1 << 32) if value & 0x80000000 else value


def _uptime_text(ms: int) -> str:
    seconds = ms // 1000
    days, rest = divmod(seconds, 86400)
    hours, rest = divmod(rest, 3600)
    minutes, secs = divmod(rest, 60)
    return (f"{days} d " if days else "") + f"{hours:02d}:{minutes:02d}:{secs:02d}"


_WIDE = {"PNOW", "STAT", "VNOW", "CNOW", "DEVI", "STIM", "PNWP", "VNWP", "CNWF", "PDNW", "FBFC", "OLLV"}


def _monitor(m: Mapping[str, int], mcur: int | None = None) -> dict[str, Any]:
    out: dict[str, Any] = {"raw": {k: f"0x{v:0{8 if k in _WIDE else 4}X}" for k, v in m.items()}}
    if "PNOW" in m:
        out["position_mm"] = round(_s32(m["PNOW"]) * R.POSITION_SCALE, 2)
    if "ALMC" in m:
        out["alarm_code"] = m["ALMC"]
    if "VNOW" in m:
        out["velocity_mm_s"] = round(_s32(m["VNOW"]) * R.VELOCITY_SCALE, 2)
    if "CNOW" in m:
        out["current"] = _s32(m["CNOW"])
        if mcur:
            out.update(rated_current=mcur, current_pct=round(_s32(m["CNOW"]) * 100 / mcur, 1))
    if "DEVI" in m:
        out["deviation"] = _s32(m["DEVI"])
    if "STIM" in m:
        out.update(uptime_ms=m["STIM"], uptime_s=round(m["STIM"] / 1000, 3), uptime=_uptime_text(m["STIM"]))
    if "POSS" in m:
        out["position_complete_no"] = m["POSS"] & 0x3FF
    if "OLLV" in m:
        out["overload_level"] = _s32(m["OLLV"])
    extra = {k: _s32(m[k]) for k in ("PNWP", "VNWP", "CNWF", "PDNW", "FBFC") if k in m}
    if extra:
        out["other"] = extra
    out["notes"] = {
        "velocity_mm_s": "VNOW, 0.01 mm/s but quantized in ~12.5 mm/s steps (VERIFIED-LIVE): not for control",
        "current": "CNOW, unit unconfirmed (643 servo-on standstill, 464 holding a 30 % push); current_pct = CNOW / "
        "MCUR (item 4.5, rated current) x 100, as IA-OS shows it",
        "deviation": "DEVI, position deviation in pulses (DOC-CON; 0 at rest)",
        "uptime": "STIM, ms since power-on (VERIFIED-LIVE)",
        "position_complete_no": "POSS (DOC-CON; read 0 after numeric moves)",
        "overload_level": "OLLV, probably the overload level in % (unconfirmed)",
        "other": "PNWP / VNWP / CNWF / PDNW / FBFC (IAI names, meanings UNVERIFIED; PNWP = -0.8 x PNOW, PDNW = "
        "+0.8 x PNOW live)",
    }
    return out


def _status_words(m: Mapping[str, int]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for name in ("DSS1", "DSS2", "DSSE", "ZONS", "SSSE", "SIPM"):
        if name in m:
            out[name] = decode_bits(m[name], BIT_TABLES[name])
    if "STAT" in m:
        out["STAT"] = decode_bits(m["STAT"], STAT_BITS, width=32)
    if "POSS" in m:
        out["POSS"] = {"value": m["POSS"], "hex": f"0x{m['POSS']:04X}", "position_no": m["POSS"] & 0x3FF,
                       "evidence": "DOC-CON (binary position number PM1..PM512)"}  # fmt: skip
    return out


def _io(m: Mapping[str, int], mode: Mapping[str, Any]) -> dict[str, Any]:
    tables = io_bit_tables(mode)
    out: dict[str, Any] = {"mode": mode.get("name"), "layout": mode.get("layout")}
    for name in ("DIPM", "DOPM"):
        if name in m:
            out[name] = decode_bits(m[name], tables[name])
    for name in ("SIPM", "ZONS"):
        if name in m:
            out[name] = decode_bits(m[name], BIT_TABLES[name])
    lamps: dict[str, Any] = {}
    if "ZONS" in m:
        zons = m["ZONS"]
        lamps.update(
            backward_end=bool(zons & R.ZONS_LS0),
            forward_end=bool(zons & R.ZONS_LS1),
            middle=bool(zons & R.ZONS_LS2),  # row 2 lights it on the 2-position gripper too (unused row)
        )
    if "ALMC" in m or "DSS1" in m:
        code, dss1 = m.get("ALMC", 0), m.get("DSS1", 0)
        fault = alarm_is_fault(code, alarm_bit=bool(dss1 & R.DSS1_ALMH), warning_bit=bool(dss1 & R.DSS1_ALML))
        lamps["alarm"] = fault  # red: ALMH, a code of level operation cancelled / cold start, or unknown without ALML
        lamps["warning"] = not fault and (bool(dss1 & R.DSS1_ALML) or (code != 0 and alarm_info(code)["warning"]))
    out["lamps"] = lamps
    out["note"] = (
        "position lamps come from ZONS b12-b14 (same on every unit tested); the DOPM layout depends on "
        "the position mode; DIPM: ST0 / ST1 of the 2-position layout are VERIFIED-LIVE (EC-GRBP8M gripper), the other "
        "bit names are unconfirmed. Alarm lamp = a fault (DSS1 ALMH, "
        "an alarm of level operation cancelled / cold start, or a code of unknown level without ALML); a known "
        "message-level code (e.g. a maintenance warning) lights the warning lamp only; the axis keeps running"
    )
    return out


def decode_monitor(
    words: Mapping[str, int], mode: Mapping[str, Any] | None = None, *, rated_current: int | None = None
) -> dict[str, Any]:
    """The live sections of :func:`unit_info` from monitor words already read (no bus access).

    ``words`` maps monitor member names to unsigned values (32-bit members combined, high word
    first), e.g. :attr:`iai_actuator_driver.ec.Status.raw` from a status poll (PNOW .. ZONS) or the 0x9000
    block. Members that are missing are left out of the result. ``mode`` is a
    :func:`position_mode` result (default: unknown, i.e. the 2-position layout, ``assumed``).
    ``rated_current`` (MCUR, item 4.5) adds ``current_pct``. Returns ``{"monitor", "status_words", "io"}``
    with the same layout as in :func:`unit_info`.
    """
    mode = position_mode(None) if mode is None else mode
    return {"monitor": _monitor(words, rated_current), "status_words": _status_words(words), "io": _io(words, mode)}


def _control_words(block: ClassBlock) -> dict[str, Any]:
    names = {
        1: "DRG1", 2: "DRG2", 3: "DRGE", 4: "POSR", 5: "DOCR", 6: "GTV", 7: "SRGE", 8: "RMDI", 9: "PEDI", 12: "RCEC",
    }  # fmt: skip
    out: dict[str, Any] = {}
    for it in block.items:
        name = names.get(it.item_id or -1, f"item {it.item_id}")
        table = BIT_TABLES.get(name)
        if table is not None:
            out[name] = {**decode_bits(it.raw, table), "address": f"0x{it.address:04X}"}
        else:
            out[name] = {"value": it.raw, "hex": f"0x{it.raw:04X}", "address": f"0x{it.address:04X}"}
    return out


# =========================================================================== alarms


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
    stride = rec.member_count * max(rec.member_size, 1)
    slots = len(words) // stride if stride else 0
    decodable = stride == R.ALARM_HISTORY_STRIDE
    used = alarm_history_slots(words) if decodable else []  # ends at the first empty slot
    history, power_ups = [], 0
    for k, code, detail, addr, stamp in used:
        if code == R.ALARM_POWER_UP:
            power_ups += 1
            continue
        record = AlarmRecord(k, code, detail, addr, stamp)
        about = alarm_info(code, absolute_encoder=absolute_encoder, detail=detail, address=addr,
                           system_code=system_code)  # fmt: skip
        when = alarm_time(stamp, rtc_in_use=rtc_in_use)
        history.append({
            **record.to_dict(), "hex": f"0x{code:03X}", "name": alarm_name(code), "newer_power_ups": power_ups,
            "level": about["level"], "group": about["group"], "detail_text": about["detail_text"],
            "clock": when["clock"], "occurred_at": when["occurred_at"],
            **({"time_after_power_on": None} if when["clock"] == "rtc" else {}),
        })  # fmt: skip
    cur: dict[str, Any] | None = None
    if current is not None:
        alarm_bit = dss1 is not None and bool(dss1 & R.DSS1_ALMH)
        warning_bit = dss1 is not None and bool(dss1 & R.DSS1_ALML)
        about = alarm_info(current, absolute_encoder=absolute_encoder, system_code=system_code)
        fault = alarm_is_fault(current, alarm_bit=alarm_bit, warning_bit=warning_bit, system_code=system_code)
        cur = {
            "code": current,
            "hex": f"0x{current:03X}",
            "name": alarm_name(current),
            "active": current != 0,
            "level": about["level"] if current else None,
            "level_text": about["level_text"] if current else None,
            "fault": fault,
            "warning": current != 0 and not fault,
            "dss1_bits": None if dss1 is None else {"ALMH": alarm_bit, "ALML": warning_bit},
            "group": about["group"] if current else None,
            "group_name": about["group_name"] if current else None,
            "reset": about["reset"],
            "home_needed": about["home_needed"],
            "remedy": about["remedy"],
            "class3_code": c3,
        }
    return {
        "current": cur,
        "history": history,
        "slots": slots,
        "alarm_count": len(history),
        "power_up_markers": power_ups,
        "empty_slots": slots - len(used) if decodable else 0,
        "absolute_encoder": absolute_encoder,
        "rtc_in_use": rtc_in_use,
        "address": f"0x{rec.data_address:04X}",
        "notes": {
            "order": "index 0 is the newest (consistent with the time stamps on hardware); the list ends at the "
            "first empty slot (code 0), so empty_slots counts the slots from there on; newer_power_ups = power-up "
            "markers at lower indices",
            "time_raw": "with the unit's calendar in use (SSSE b8 = 1, rtc_in_use): seconds since 2000-01-01 "
            "00:00:00, shown as occurred_at (clock 'rtc'; DOC ME0162-12A p.4-10); otherwise the time since power-on "
            "(clock 'power_on'): milliseconds on the EC (VERIFIED; it has no clock), time_after_power_on",
            "names": "IAI's controller alarm names, as IA-OS shows them",
            "levels": "message (a known warning code: the axis keeps running), operation cancelled (alarm reset), "
            "cold start (software reset / power cycle), unknown (a fault unless ALML is set); UNVERIFIED except "
            "0x090 and 0x0EE on an absolute unit (VERIFIED); 0x0EE depends on absolute_encoder (SUPT ABSE / SUP2 "
            "MABS); remedies are the driver's own words",
        },
    }


# =========================================================================== positions

_ROW_ROLES_3POS = {0: "backward end", 1: "forward end", 2: "middle"}
_ROW_ROLES_2POS = {0: "backward end", 1: "forward end", 2: "unused in 2-position mode"}
# Simple-data row roles (UNVERIFIED): 262 / 263 are the PREVIOUS backward / forward %, the Middle % row is 266.
_SIMPLE_ROLES = dict(R.SIMPLE_DATA_ROLES)
_SIMPLE_ROLES_3POS = _SIMPLE_ROLES


def _row(n: int, address: int, words: Sequence[int], role: str) -> dict[str, Any]:
    e = PositionEntry.from_registers(words)
    push = bool(e.control_flags & R.CTLF_PUSH)
    away = bool(e.control_flags & R.CTLF_PUSH_AWAY)
    return {
        "n": n,
        "role": role,
        "address": f"0x{address:04X}",
        "position_mm": e.position_mm,
        "band_mm": e.band_mm,
        "speed_mm_s": e.speed_mm_s,
        "zone_plus_mm": e.zone_plus_mm,
        "zone_minus_mm": e.zone_minus_mm,
        "accel_g": e.accel_g,
        "decel_g": e.decel_g,
        "push_power": e.push_power,
        "push_pct": round(e.push_power * 100 / 255, 1),
        "load_threshold": e.load_threshold,
        "control_flags": f"0x{e.control_flags:04X}",
        "ctlf_on": decode_bits(e.control_flags, CTLF_BITS)["on"],
        "push": push,
        "push_direction": ("away from home" if away else "toward home") if push else None,
        "push_end_mm": round(e.position_mm + (e.band_mm if away else -e.band_mm), 2) if push else None,
        "checksum": None if e.checksum is None else f"0x{e.checksum:04X}",
        "checksum_ok": e.checksum_ok,
        "raw": " ".join(f"{w:04X}" for w in words),
    }


def _simple_row(n: int, address: int, words: Sequence[int], roles: Mapping[int, str]) -> dict[str, Any]:
    return {
        "n": n,
        "role": roles.get(n, "?"),
        "address": f"0x{address:04X}",
        "position_mm": round(R.from_i32_words(words[0], words[1]) * R.POSITION_SCALE, 2),
        "band_mm": round(R.from_u32_words(words[2], words[3]) * R.POSITION_SCALE, 2),
        "speed_pct": R.from_u32_words(words[4], words[5]),
        "accel_pct": words[10],
        "decel_pct": words[11],
        "push_power": words[12],
        "control_flags": f"0x{words[14]:04X}",
        "checksum_ok": R.position_checksum(words) == words[15],
        "raw": " ".join(f"{w:04X}" for w in words),
    }


def _direct_move(words: Sequence[int]) -> dict[str, Any]:
    out: dict[str, Any] = {
        "address": "0x9900",
        "target_mm": round(R.from_i32_words(words[0], words[1]) * R.POSITION_SCALE, 2),
        "band_mm": round(R.from_u32_words(words[2], words[3]) * R.POSITION_SCALE, 2),
        "speed_mm_s": round(R.from_u32_words(words[4], words[5]) * R.VELOCITY_SCALE, 2),
        "accel_g": round(words[6] * R.ACCEL_SCALE, 2) if len(words) > 6 else None,
    }
    if len(words) > 8:
        out.update(
            push_power=words[7],
            push_pct=round(words[7] * 100 / 255, 1),
            control_flags=f"0x{words[8]:04X}",
            ctlf_on=decode_bits(words[8], CTLF_BITS)["on"],
        )
    if len(words) > 9:
        out["drg1_mirror"] = f"0x{words[9]:04X}"
    out["raw"] = " ".join(f"{w:04X}" for w in words)
    out["note"] = "the last values written by any client (volatile; CTLF clears after each move start)"
    return out


def _active_profile(words: Sequence[int]) -> dict[str, Any]:
    out: dict[str, Any] = {"address": "0x0F00", "raw": " ".join(f"{w:04X}" for w in words)}
    if len(words) >= 13:
        out.update(
            target_mm=round(R.from_i32_words(words[0], words[1]) * R.POSITION_SCALE, 2),
            speed_mm_s=round(R.from_u32_words(words[2], words[3]) * R.VELOCITY_SCALE, 2),
            accel_g=round(R.from_u32_words(words[4], words[5]) * R.ACCEL_SCALE, 2),
            decel_g=round(R.from_u32_words(words[6], words[7]) * R.ACCEL_SCALE, 2),
            band_mm=round(R.from_u32_words(words[8], words[9]) * R.POSITION_SCALE, 2),
            spow=words[10],
            dpow=words[11],
            push_power=words[12],
        )
    if len(words) >= 14:
        out["control_flags"] = f"0x{words[13]:04X}"
    out["note"] = "the move profile the EC applied (IAI names; VERIFIED-LIVE values in one snapshot)"
    return out


# =========================================================================== identity


def _identification_extra(words: Sequence[int]) -> dict[str, Any]:
    """Fields of 0x0600 besides serial / model, with IAI's field names (meanings UNVERIFIED)."""
    if len(words) < 0x1A:
        return {}
    date = words[0x14:0x16]
    date_text = f"{date[0]:04X}-{date[1] >> 8:02X}-{date[1] & 0xFF:02X}"
    prdt = R.from_u32_words(words[R.IDENT_PRDT], words[R.IDENT_PRDT + 1])
    accl = R.from_u32_words(words[R.IDENT_ACCL], words[R.IDENT_ACCL + 1])
    level = extensibility_level(accl)
    level_names = {0: "Normal", 1: "Position3", 2: "Extensibility phase 1", 3: "Extensibility phase 2",
                   4: "Multi-I/O public release"}  # fmt: skip
    return {
        "actuator_serial": R.words_to_ascii(words[0x08:0x10]) or None,
        "second_id": R.words_to_ascii(words[0x08:0x10]) or None,  # compatibility alias of actuator_serial
        "manufacture_info": {"hex": f"{prdt:08X}", "mrev": f"{prdt >> 16:04X}", "frev": f"{prdt & 0xFFFF:04X}",
                             "ascii": R.words_to_ascii(words[0x10:0x12]) or None},  # fmt: skip
        "code": R.words_to_ascii(words[0x10:0x12]) or None,  # compatibility alias (PRDT as text)
        "accl": {"hex": f"0x{accl:08X}", "extensibility_level": level, "level_name": level_names.get(level, "?")},
        "bisd_raw": f"{R.from_u32_words(words[0x14], words[0x15]):08X}",
        "user_name": R.words_to_ascii(words[0x1A:0x20]) or None,
        "words_0612": [f"{w:04X}" for w in words[0x12:0x18]],
        "date_guess": date_text if all(c.isdigit() for c in date_text.replace("-", "")) else None,
        "note": "IAI field names (meanings UNVERIFIED): 0x0608 ACS actuator serial, 0x0610 PRDT manufacture "
        "information (MREV + FREV), 0x0612 ACCL (bits 8-11 extensibility level), 0x0614 BISD (meaning unknown; the "
        "date reading is a guess), 0x061A ANA1-3 the user-editable unit name (not a model field)",
    }


def _bits_on(value: int, names: Mapping[int, str], width: int = 32) -> list[str]:
    return [names.get(b, f"b{b}") for b in range(width) if value >> b & 1]


def _capabilities(
    classes: Mapping[int, ClassBlock], support: Mapping[str, int] | None, limits: Any
) -> dict[str, Any]:
    """Identity extras: support bits, axis limits, link, servo monitor, collision (no I/O)."""

    def item(ci: int, item_id: int) -> ClassItem | None:
        block = classes.get(ci)
        return None if block is None else block.item(item_id)

    out: dict[str, Any] = {"support": decode_support_words(support)}
    if limits is not None:
        lim = limits.to_dict()
        if limits.max_push_pct:
            lim["push_force_at_max_estimate"] = limits.push_force_estimate(limits.max_push_pct)
            lim["push_force_note"] = "estimate TCPC x % / MXPP + TCOA; unit probably N (UNVERIFIED)"
        kind = limits.actuator_kind
        lim["actuator_kind_name"] = None if kind is None else R.ACTUATOR_KINDS.get(kind, "unknown")
        out["axis_limits"] = lim
    pshv = item(R.CATEGORY_MOTION_PARAMS, R.PARAM_ITEM_PSHV)
    if pshv is not None:
        out["push_speed_mm_s"] = round(pshv.signed * R.VELOCITY_SCALE, 2)
    brsl, rtim = item(R.CATEGORY_MOTION_PARAMS, R.PARAM_ITEM_BRSL), item(R.CATEGORY_MOTION_PARAMS, R.PARAM_ITEM_RTIM)
    out["link"] = {
        "boot_baud_code": None if brsl is None else brsl.raw,
        "boot_baudrate": None if brsl is None else _baud_by_code(brsl.raw),
        "transmit_delay_ms": None if rtim is None else rtim.raw,
        "note": "BRSL (item 8.3) = the power-up rate (volatile changes return to it after a power cycle or software "
        "reset); RTIM (8.4) = minimum delay before the EC's transmitter turns on (UNVERIFIED)",
    }
    fmnt = item(R.CATEGORY_MOTION_PARAMS, R.PARAM_ITEM_FMNT)
    modes = {0: "off", 1: "4 channels", 2: "8 channels", 3: "2 channels"}
    out["servo_monitor"] = None if fmnt is None else {
        "fmnt_hex": f"0x{fmnt.raw:08X}", "mode": fmnt.raw & 0xF, "mode_name": modes.get(fmnt.raw & 0xF, "?"),
        "buffering_cycle": fmnt.raw >> 16,
        "note": "FMNT (item 8.54): high-speed sampling needs mode != 0, a non-volatile change (not done by this "
        "driver); 0x46 is not used on the EC",
    }  # fmt: skip
    flags = (out["support"] or {}).get("flags", {})
    present = [i for i in R.COLLISION_ITEMS if item(R.CATEGORY_MOTION_PARAMS, i) is not None]
    sets = []
    for n in range(3):
        cdc, cdd, cdb = (item(R.CATEGORY_MOTION_PARAMS, 119 + 3 * n + k) for k in range(3))
        if cdc is None and cdd is None and cdb is None:
            continue
        evac = None if cdb is None else cdb.raw & 0xFFFF
        sets.append({
            "set": n + 1,
            "current_pct": None if cdc is None else round(cdc.raw * 100 / 255),
            "distance_mm": None if cdd is None else round(cdd.signed * R.POSITION_SCALE, 2),
            "time_ms": None if cdb is None else cdb.raw >> 16,
            "evacuation_position": None if evac is None else (-1 if evac == 0xFFFF else evac),
            "raw": [None if i is None else i.hex for i in (cdc, cdd, cdb)],
        })  # fmt: skip
    spio = item(R.CATEGORY_MOTION_PARAMS, R.PARAM_ITEM_SPIO)
    supported = flags.get("collision_detection")
    out["collision"] = {
        "supported": supported,
        "reported": "supported (MVPT b13 CDAS = 1)" if supported else (
            "not reported (MVPT b13 CDAS = 0)" if supported is False else "unknown (class 1 not readable)"),
        "vibration_control": flags.get("vibration_control"),
        "parameter_items": [f"8.{i}" for i in present],
        "sets": sets,
        "output_on_collision": None if spio is None or not present else (
            "warning" if spio.raw & R.SPIO_SLCD else "alarm"),
        "note": "READ-ONLY: collision sets are class 8 items 119-127 (current % = raw x 100 / 255, distance 0.01 mm, "
        "time = CDB b16-31 ms, evacuation position = CDB b0-15; layout UNVERIFIED); never written by this driver",
    }  # fmt: skip
    return out


def _baud_by_code(code: int) -> int | None:
    from iai_actuator_driver.modbus import BAUD_BY_CODE  # absolute: runs re-bound in info.py too

    return BAUD_BY_CODE.get(code)


def read_support_words(ec: Elecylinder, records: Sequence[IndexCategory] | None = None) -> dict[str, int] | None:
    """Class 1 support-function words by name (SUPT, DIAG, MVPT, SPTS, CTYP, SUP2, ... SUP3). FC03 only.

    Class 1 has no ID map and a fixed member order (``registers.SUPPORT_MEMBERS``); the directory gives
    the member count (EC-R6S rod: 6, up to SUP2; slider / gripper 11, up to SUP3). ``None`` when the class is not
    in the directory. An exception response propagates (:class:`ModbusException`)."""
    rec = next((c for c in _records(ec, records) if c.category_id == R.CATEGORY_SUPPORT), None)
    if rec is None or rec.member_count < 1:
        return None
    members = R.SUPPORT_MEMBERS[: rec.member_count]
    words = ec.read_registers(rec.data_address, sum(size for _, size in members))
    out: dict[str, int] = {}
    offset = 0
    for name, size in members:
        out[name] = words[offset] if size == 1 else (words[offset] << 16) | words[offset + 1]
        offset += size
    return out


def read_accl(ec: Elecylinder, records: Sequence[IndexCategory] | None = None) -> int | None:
    """ACCL (0x0612, u32; bits 8-11 = extensibility level) from the identification block. FC03 only."""
    rec = next((c for c in _records(ec, records) if c.category_id == R.CATEGORY_IDENTIFICATION), None)
    if rec is None:
        return None
    words = ec.read_registers(rec.data_address + R.IDENT_ACCL, 2)
    return R.from_u32_words(words[0], words[1])


# =========================================================================== unit_info


def unit_info(ec: Elecylinder, *, sections: Iterable[str] | None = None) -> dict[str, Any]:
    """Everything the EC tells about itself, as one JSON-serialisable dict. Read-only (FC03 + FC11).

    ``sections`` limits the work to some of :data:`SECTIONS` (default: all). The returned dict has
    these keys (a section not requested is ``None``):

    * ``meta``: ``schema``, ``read_only`` (True), ``function_codes`` (["03", "11"]), ``taken_at``
      (ISO time), ``duration_s``, ``driver_version``, ``sections``.
    * ``identity``: ``model``, ``model_name``, ``user_name`` (0x061A, IA-OS's user-editable unit name;
      ``short_model`` is a compatibility alias), ``serial_number``, ``actuator_serial``, ``controller``,
      ``system_code`` / ``system_name`` / ``run_status`` / ``unit_code`` / ``unit_name`` (the EC family name) /
      ``firmware`` / ``slave_id_raw`` (FC11), ``identity_source`` (FC11, or CTCL 0x0300 when FC11 is silent),
      ``unit_known`` (the driver has a family name for this unit code; display only), ``product_family`` (EC,
      PCON, ACON/DCON, SCON or "IAI controller"), ``controller_name`` (CON: 0x8010), ``units`` (mm / deg),
      ``kind``, ``max_positions``, ``actuator_kind`` / ``actuator_kind_name`` / ``actuator`` (AKID 7.33), ``rotary``, ``axis_number`` (AXNO 0x8004), ``accl`` / ``extensibility_level``, ``slave``,
      ``baudrate``, ``baud_detected``, ``soft_limits_mm``, ``stroke_mm`` (EC stroke), ``configured_stroke_mm``,
      ``effective_stroke_mm``, ``ec_max_speed_mm_s`` (MAXV 7.4) / ``ec_max_speed_source``, ``ec_max_accel_g`` /
      ``ec_max_decel_g`` (MAXA / MAXD 7.5 / 7.6), ``max_accel_g`` (the configured cap), ``max_decel_g`` (MAXD),
      ``speed_cap_mm_s``, ``accel_cap_g``,
      ``push_cap_pct``, ``position_mode``, ``identification_extra``, ``capabilities`` (``support``: the class-1
      words decoded with ``flags`` push / relative_moves / software_reset / collision_detection (MVPT b13 CDAS) /
      vibration_control / wireless_module / absolute_encoder / uses_function_0x48; ``axis_limits``
      (:class:`AxisLimits` + push force estimate); ``push_speed_mm_s`` (PSHV 8.15); ``link`` (BRSL 8.3 power-up
      baud, RTIM 8.4); ``servo_monitor`` (FMNT 8.54); ``collision`` (CDAS, class-8 items 119-127 decoded read-only when present,
      SLCD output)), ``directory`` (:func:`directory_summary`), ``notes``.
    * ``parameters``: ``named`` (P1-P18: ``no``, ``name``, ``key``, ``address``, ``raw_hex``,
      ``value``, ``unit``, ``text``, ``encoding``, ``evidence``, ``found``, ``read_only``, plus the per-model
      flags of :func:`annotate_parameters`), ``model`` (:func:`model_parameter_list` with the
      :func:`parameter_visibility` rules: ``listed`` = what IA-OS's screen shows, ``hidden`` with the
      reasons), ``not_located`` (empty: P8 / P11 are located), ``classes`` (every item of classes 3 / 4 /
      7 / 8 / 9 / 0x0C: :meth:`ClassItem.to_dict` plus ``params`` and ``note``) and ``write_note``.
    * ``monitor``: ``position_mm``, ``alarm_code``, ``velocity_mm_s``, ``current`` (+ ``rated_current`` /
      ``current_pct`` = CNOW / MCUR x 100 when MCUR 4.5 was read), ``deviation``, ``uptime_ms`` /
      ``uptime_s`` / ``uptime``, ``position_complete_no``, ``overload_level``, ``other``, ``raw``, ``notes``.
    * ``status_words``: DSS1, DSS2, DSSE, ZONS, STAT (32 bit), SSSE, SIPM as :func:`decode_bits`
      dicts, POSS as a number.
    * ``io``: ``mode``, ``layout``, DIPM / DOPM (tables for the position mode), SIPM, ZONS, ``lamps``
      (``backward_end``, ``forward_end``, ``middle``, ``alarm`` = a fault, ``warning`` = a message-level alarm).
    * ``control_words``: DRG1 / DRG2 / DRGE decoded, POSR and the other 0x0D00 members raw.
    * ``alarms``: ``current`` (``code``, ``hex``, ``name``, ``active``, ``level`` / ``level_text``
      (:func:`iai_actuator_driver.alarms.alarm_level`: message / operation_cancelled / cold_start / unknown), ``fault``,
      ``warning``, ``dss1_bits``, ``group`` / ``group_name``, ``reset``, ``home_needed``, ``remedy`` (the
      driver's own words), ``class3_code``), ``history`` (alarms only, index 0 = newest, up to the first empty
      slot: ``index``, ``code``, ``hex``, ``name``, ``level``, ``group``, ``detail``, ``address``,
      ``detail_text``, ``time_raw`` / ``time_ms`` (ms since power-on), ``time_after_power_on``,
      ``newer_power_ups``), ``slots``, ``alarm_count``, ``power_up_markers``, ``empty_slots`` (the slots from
      the first empty one on), ``absolute_encoder`` (SUPT ABSE / SUP2 MABS; picks the 0x0EE level, ``None`` =
      unknown).
    * ``maintenance``: ``total_moves``, ``distance_m``, ``items`` (``id``, ``symbol``, ``meaning``,
      ``confirmed``, ``address``, ``hex``, ``value``, ``unset``).
    * ``positions``: ``rows`` (0-2), ``simple_data_rows`` (260-263 present / previous % values of IA-OS's simple
      data page, and 266 the Middle % in 3-position mode), ``direct_move`` (0x9900 block), ``active_profile``
      (0x0F00).
    * ``errors``: ``"section: message"`` for every part that could not be read.

    A link that does not answer at all raises :class:`ModbusTimeout`; once the EC has answered, read
    problems are reported in ``errors`` and a later timeout ends the remaining sections.
    """
    from iai_actuator_driver import __version__  # absolute: runs re-bound in info.py too

    wanted = tuple(SECTIONS) if sections is None else tuple(dict.fromkeys(sections))
    unknown = [s for s in wanted if s not in SECTIONS]
    if unknown:
        raise ValueError(f"unknown section(s) {unknown}; choose from {list(SECTIONS)}")
    started = time.perf_counter()
    result: dict[str, Any] = {
        "meta": {
            "schema": SCHEMA,
            "read_only": True,
            "function_codes": ["03", "11"],
            "taken_at": _dt.datetime.now(_dt.timezone.utc).astimezone().isoformat(timespec="seconds"),
            "driver_version": __version__,
            "sections": list(wanted),
        },
        **{key: None for key in ("identity", "parameters", "monitor", "status_words", "io", "control_words",
                                 "alarms", "maintenance", "positions")},  # fmt: skip
        "errors": [],
    }
    errors: list[str] = result["errors"]
    ec.read_registers(R.PNOW, 2)  # link probe: a timeout here propagates (nothing answers)

    state: dict[str, Any] = {"header": None, "records": (), "info": None, "classes": {}, "monitor": {},
                             "support": None}  # fmt: skip
    link_lost = False

    def run(name: str, fn: Callable[[], None]) -> None:
        nonlocal link_lost
        if link_lost:
            errors.append(f"{name}: skipped (link lost)")
            return
        try:
            fn()
        except ModbusTimeout as exc:
            link_lost = True
            errors.append(f"{name}: no answer ({exc})")
        except (EcDriverError, LookupError, ValueError) as exc:
            errors.append(f"{name}: {type(exc).__name__}: {exc}")

    def directory() -> None:
        if "identity" in wanted:
            info = ec.identify()
            state["info"] = info
            state["header"], state["records"] = info.index_header, info.categories
            if not info.categories:
                raise LookupError("index table not readable: " + "; ".join(info.errors))
        else:
            state["header"], state["records"] = read_directory(ec)

    run("directory", directory)
    records: Sequence[IndexCategory] = state["records"]

    def classes() -> None:
        if {"parameters", "identity"} & set(wanted):
            want: tuple[int, ...] = PARAMETER_CLASSES
        else:
            want = (R.CATEGORY_MOTION_PARAMS, *((R.CATEGORY_MONITOR2,) if "monitor" in wanted else ()))
        for ci in want:
            try:
                state["classes"][ci] = read_class(ec, ci, records=records)
            except ModbusException as exc:
                errors.append(f"parameters: class 0x{ci:04X}: {exc}")
            except LookupError as exc:
                errors.append(f"parameters: {exc}")
        if {"parameters", "identity"} & set(wanted):
            try:
                state["support"] = read_support_words(ec, records)
            except ModbusException as exc:
                errors.append(f"identity: class 1 support words: {exc}")

    if records and ({"parameters", "identity", "monitor", "positions"} & set(wanted)):
        run("parameters", classes)
    p9_block = state["classes"].get(R.POSITION_MODE_ITEM[0])
    p9_item = p9_block.item(R.POSITION_MODE_ITEM[1]) if p9_block is not None else None
    info = state["info"]
    mode = position_mode(p9_item.raw if p9_item is not None else None, info.model if info is not None else None)

    def _system_code() -> int | None:
        if info is not None and info.system_code is not None:
            return int(info.system_code)
        code = getattr(ec, "system_code", None)
        return code if isinstance(code, int) else None

    def rtc(monitor: Mapping[str, int]) -> bool | None:
        """SSSE b8 (calendar in use), read once: from the monitor words, else 0x9015."""
        if "rtc" not in state:
            state["rtc"] = rtc_in_use(_read_ssse(ec, monitor or {}))
        return state["rtc"]

    if "identity" in wanted and info is not None:

        def identity() -> None:
            data = info.to_dict()
            data.pop("categories", None)
            data.pop("index_header", None)
            data.pop("errors", None)
            extra: dict[str, Any] = {}
            try:
                extra = _identification_extra(read_block(ec, R.CATEGORY_IDENTIFICATION, records=records))
            except (ModbusException, LookupError, ValueError) as exc:
                errors.append(f"identity: identification block: {exc}")
            limits = getattr(ec, "axis_limits", None)
            data.update(
                slave=ec.slave,
                baudrate=ec.baudrate,
                baud_detected=ec.baud_detected,
                stroke_mm=_pair(ec.ec_stroke),
                configured_stroke_mm=_pair(ec.stroke),
                effective_stroke_mm=_pair(ec.effective_stroke),
                ec_max_speed_mm_s=ec.ec_max_speed_mm_s,
                ec_max_speed_source=getattr(ec, "ec_max_speed_note", None),
                ec_max_accel_g=info.max_accel_g,
                ec_max_decel_g=info.max_decel_g,
                speed_cap_mm_s=ec.speed_cap_mm_s,
                max_accel_g=ec.max_accel_g,
                accel_cap_g=getattr(ec, "accel_cap_g", ec.max_accel_g),
                push_cap_pct=getattr(ec, "push_cap_pct", None),
                units=info.units,
                kind=info.kind,
                product_family=info.product_family,
                position_mode=mode,
                identification_extra=extra,
                capabilities=_capabilities(state["classes"], state["support"], limits),
                directory=directory_summary(state["header"], records),
                notes=list(info.errors),
            )
            result["identity"] = data

        run("identity", identity)

    if "parameters" in wanted and state["classes"]:

        def parameters() -> None:
            blocks = state["classes"]
            system_code = None
            if info is not None:
                unit_code, model_name, system_code = info.unit_code, info.model_name, info.system_code
            elif hasattr(ec, "parameter_model"):
                found = ec.parameter_model()  # FC11 / CTCL + the 0x0600 model string (read-only)
                unit_code, model_name = found["unit_code"], found["model_name"]
                system_code = found.get("system_code")
            else:
                unit_code, model_name = getattr(ec, "unit_code", None), None
            accl = info.accl if info is not None and info.accl is not None else None
            if accl is None:
                try:
                    accl = read_accl(ec, records)
                except ModbusException as exc:
                    errors.append(f"parameters: ACCL (0x0612): {exc}")
            visibility = parameter_visibility(blocks, state["support"], accl)
            model = model_parameter_list(unit_code, model_name, system_code=system_code, visibility=visibility)
            not_located = []
            for no, data in NOT_LOCATED_PARAMETERS.items():
                listed = model["known"] and no in model["listed"]
                not_located.append({"no": no, "name": data["name"], "listed": listed,
                                    "iaos_text": data["iaos_text"] if listed else "", "note": data["note"]})
            named = annotate_parameters(decode_user_parameters(blocks), unit_code, model_name,
                                        system_code=system_code, visibility=visibility)  # fmt: skip
            numbered: list[dict[str, Any]] = []
            if system_code is not None and not R.is_ec_family(system_code) and hasattr(ec, "parameters"):
                try:
                    numbered = ec.parameters()  # the CON parameter numbers this unit's ID map has
                except (ModbusException, LookupError, ValueError) as exc:
                    errors.append(f"parameters: numbered CON parameters: {exc}")
                named = []  # the EC P-numbers do not apply to a CON controller
            range_check: dict[str, Any] | None = None
            if numbered:
                family = R.product_family(system_code)
                limits = con_params.con_unit_limits(blocks, state["support"])
                numbered = annotate_con_parameters(numbered, family)
                range_check = con_range_check(numbered, family, limits)
            result["parameters"] = {
                "named": named,
                "numbered": numbered,
                "range_check": range_check,
                "model": model,
                "not_located": not_located,
                "classes": [_annotated_class(blocks[ci]) for ci in PARAMETER_CLASSES if ci in blocks],
                "note": "named parameters: P1-P7, P9, P10, P12 " + _PARAM_EVIDENCE + "; P3 decoded by the parity of "
                "the ORG home pattern; P8, P11, P13-P18 at UNVERIFIED locations and read-only; listed = what IA-OS's "
                "screen shows (parameter_visibility rules); every item is also listed raw by class",
                "write_note": PARAM_COMMIT_NOTE,
            }

        run("parameters", parameters)

    if "monitor" in wanted:

        def monitor() -> None:
            try:
                words = read_block(ec, R.CATEGORY_MONITOR, records=records)
            except LookupError:
                words = ec.read_registers(R.MONITOR_BASE, R.STATUS_BLOCK_WORDS)  # verified on both units
            m = _split_monitor(words)
            state["monitor"] = m
            c4 = state["classes"].get(R.CATEGORY_MONITOR2)
            mcur = c4.item(R.MONITOR2_ITEM_MCUR) if c4 is not None else None
            result.update(decode_monitor(m, mode, rated_current=None if mcur is None else mcur.raw))

        def control() -> None:
            result["control_words"] = _control_words(read_class(ec, R.CATEGORY_IO_CONTROL, records=records))

        run("monitor", monitor)
        if records:
            run("control_words", control)

    if "alarms" in wanted and records:

        def alarms() -> None:
            rec = _record(records, R.CATEGORY_ALARM_HISTORY)
            words = read_block(ec, R.CATEGORY_ALARM_HISTORY, records=records)
            m = state["monitor"] or _split_monitor(ec.read_registers(R.MONITOR_BASE, R.STATUS_BLOCK_MIN_WORDS))
            c3_block = state["classes"].get(R.CURRENT_ALARM_ITEM[0])
            if c3_block is None:
                try:
                    c3_block = read_class(ec, R.CURRENT_ALARM_ITEM[0], records=records)
                except (ModbusException, LookupError):
                    c3_block = None
            c3 = c3_block.item(R.CURRENT_ALARM_ITEM[1]) if c3_block is not None else None
            if state["support"] is None:  # for the 0x0EE level (absolute vs incremental unit)
                try:
                    state["support"] = read_support_words(ec, records)
                except ModbusException:
                    pass  # absolute_encoder stays unknown
            absolute = absolute_encoder_from_support(state["support"])
            result["alarms"] = decode_alarm_history(rec, words, m.get("ALMC"), m.get("DSS1"),
                                                    None if c3 is None else c3.raw, absolute, rtc_in_use=rtc(m),
                                                    system_code=_system_code())  # fmt: skip

        run("alarms", alarms)

    if "maintenance" in wanted and records:

        def maintenance() -> None:
            family = R.product_family(_system_code())
            result["maintenance"] = decode_maintenance(read_class(ec, R.CATEGORY_MAINTENANCE, records=records),
                                                       rtc=rtc(state["monitor"]), family=family)  # fmt: skip

        run("maintenance", maintenance)

    if "positions" in wanted and records:

        def positions() -> None:
            three = mode.get("positions") == 3
            roles = _ROW_ROLES_3POS if three else _ROW_ROLES_2POS
            simple_roles = _SIMPLE_ROLES_3POS if three else _SIMPLE_ROLES
            table = _record(records, R.CATEGORY_POSITION_TABLE)
            stride = R.POSITION_STRIDE
            out: dict[str, Any] = {"mode": mode.get("name"), "rows": [], "simple_data_rows": []}
            words = read_block(ec, R.CATEGORY_POSITION_TABLE, 0, 3 * stride, records=records)
            for n in range(3):
                row_words = words[n * stride : (n + 1) * stride]
                out["rows"].append(_row(n, table.data_address + n * stride, row_words, roles[n]))
            first = R.POSITION_SIMPLE_DATA_ROWS.start + 4  # rows 260-263: present / previous % (Bwd, Fwd)
            words = read_block(ec, R.CATEGORY_POSITION_TABLE, first * stride, 4 * stride, records=records)
            for i in range(4):
                n = first + i
                out["simple_data_rows"].append(
                    _simple_row(n, table.data_address + n * stride, words[i * stride : (i + 1) * stride], simple_roles)
                )
            if three:  # the Middle % row is 266, not 263 (UNVERIFIED)
                n = 266
                words = read_block(ec, R.CATEGORY_POSITION_TABLE, n * stride, stride, records=records)
                out["simple_data_rows"].append(_simple_row(n, table.data_address + n * stride, words, simple_roles))
            out["simple_data_note"] = (
                "rows of IA-OS's 'simple data' page: 260 / 261 present % (Bwd / Fwd), 262 / 263 the PREVIOUS % "
                "values, 266 the Middle present % (roles UNVERIFIED); speed = % of MAXV (7.4), accel / decel (+0x0A / "
                "+0x0B) = % of MAXA / MAXD (7.5 / 7.6), floor on writing. IA-OS shows these rows, not rows 0-2, and "
                "they are not updated when rows 0-2 are written (VERIFIED); an IA-OS Transfer is expected to rewrite "
                "rows 0-2 from them (UNVERIFIED); never written by this driver"
            )
            try:
                out["direct_move"] = _direct_move(read_block(ec, R.CATEGORY_COMMAND2, records=records))
            except (ModbusException, LookupError, ValueError) as exc:
                errors.append(f"positions: 0x9900 block: {exc}")
            try:
                out["active_profile"] = _active_profile(read_block(ec, R.CATEGORY_ACTIVE_PROFILE, records=records))
            except (ModbusException, LookupError, ValueError) as exc:
                errors.append(f"positions: 0x0F00 block: {exc}")
            result["positions"] = out

        run("positions", positions)

    for name in wanted:
        if name != "identity" and not records and name not in ("monitor",):
            errors.append(f"{name}: skipped (no directory)")
    result["meta"]["duration_s"] = round(time.perf_counter() - started, 3)
    return result


def _pair(value: tuple[float, float] | None) -> list[float] | None:
    return None if value is None else [value[0], value[1]]


# =========================================================================== read-only backup

BACKUP_SCHEMA = "ec_driver.backup/1"
BACKUP_ROWS = ("used", "all")


def _row_selection(ec: Elecylinder, table: IndexCategory, rows: str | Iterable[int]) -> list[int]:
    """The position rows a backup reads: ``"used"`` = rows 0 .. max_positions() - 1 plus, on an EC, the simple-data
    rows (256-267) inside the table; ``"all"`` = every row of the directory record; else the given numbers."""
    count = table.list_count or 0
    if isinstance(rows, str):
        if rows == "all":
            return list(range(count))
        if rows != "used":
            raise ValueError(f"rows must be one of {BACKUP_ROWS} or row numbers, got {rows!r}")
        used = set(range(min(ec.max_positions(), count)))
        if ec.is_ec:
            used |= {n for n in R.POSITION_SIMPLE_DATA_ROWS if n < count}
        return sorted(used)
    chosen = sorted({int(n) for n in rows})
    bad = [n for n in chosen if not 0 <= n < count]
    if bad:
        raise ValueError(f"rows {bad} are outside 0..{count - 1} (the directory's position table)")
    return chosen


def _spans(numbers: Sequence[int]) -> list[tuple[int, int]]:
    """Contiguous ``(first, count)`` runs of sorted numbers."""
    spans: list[tuple[int, int]] = []
    for n in numbers:
        if spans and spans[-1][0] + spans[-1][1] == n:
            spans[-1] = (spans[-1][0], spans[-1][1] + 1)
        else:
            spans.append((n, 1))
    return spans


def _backup_rows(ec: Elecylinder, records: Sequence[IndexCategory], rows: str | Iterable[int]) -> dict[str, Any]:
    table = _record(records, R.CATEGORY_POSITION_TABLE)
    stride = table.member_count * max(table.member_size, 1)
    chosen = _row_selection(ec, table, rows)
    out: list[dict[str, Any]] = []
    for first, count in _spans(chosen):
        words = read_block(ec, R.CATEGORY_POSITION_TABLE, first * stride, count * stride, records=records)
        for i in range(count):
            n, row = first + i, words[i * stride : (i + 1) * stride]
            entry: dict[str, Any] = {"n": n, "address": f"0x{table.data_address + n * stride:04X}",
                                     "words": [f"{w:04X}" for w in row]}  # fmt: skip
            if stride == R.POSITION_STRIDE:
                e = PositionEntry.from_registers(row)
                entry["decoded"] = {**{k: getattr(e, k) for k in e.__dataclass_fields__}, "checksum_ok": e.checksum_ok}
            out.append(entry)
    selection = rows if isinstance(rows, str) else "list"
    return {"stride_words": stride, "rows_in_table": table.list_count, "selection": selection, "rows": out}


def backup(ec: Elecylinder, path: str | Path | None = None, *, rows: str | Iterable[int] = "used") -> dict[str, Any]:
    """A read-only backup of one axis as a JSON-serialisable dict (FC03 + FC11 only; nothing is written to the unit).

    Contents: ``meta`` (``schema`` :data:`BACKUP_SCHEMA`, ``read_only``, ``taken_at``, ``driver_version``),
    ``identity`` (the identify() fields, slave, baud rate), ``directory`` (:func:`directory_summary`: the addresses
    a later restore must find unchanged), ``support`` (class-1 words) and ``accl``, ``parameters`` (every item of the
    parameter classes 3 / 4 / 7 / 8 / 9 / 0x0C with class, item ID, entry, address and raw value; plus ``named``:
    the EC user parameters, or on a CON unit the numbered parameters decoded for its family), ``positions`` (raw
    words and the decoded entry per row; ``rows``: ``"used"`` = the rows the unit uses plus an EC's simple-data rows
    256-267, ``"all"`` = every row of the directory (an EC has 768: slow), or row numbers), ``alarms`` (the raw
    history block and the decoded entries, time stamps per :func:`iai_actuator_driver.alarms.alarm_time`), ``maintenance``
    (class 0x200 counters) and ``errors`` (``"section: message"`` per part that could not be read).

    ``path``: also written there as UTF-8 JSON (the dict then carries ``meta.path``). A unit that does not answer
    at all raises :class:`ModbusTimeout`; later read problems go to ``errors``. Restoring is not part of the driver
    (a restore writes non-volatile memory). This is the read side of the rule "back up a unit before non-volatile
    writes" (like the PC tool's "Save all data", ME0155-31A p.40).
    """
    from iai_actuator_driver import __version__  # absolute: runs re-bound in info.py too

    started = time.perf_counter()
    errors: list[str] = []
    out: dict[str, Any] = {
        "meta": {"schema": BACKUP_SCHEMA, "read_only": True, "function_codes": ["03", "11"],
                 "taken_at": _dt.datetime.now(_dt.timezone.utc).astimezone().isoformat(timespec="seconds"),
                 "driver_version": __version__,
                 "rows": rows if isinstance(rows, str) else sorted({int(n) for n in rows})},
        **{key: None for key in ("identity", "directory", "support", "accl", "parameters", "positions", "alarms",
                                 "maintenance")},
        "errors": errors,
    }  # fmt: skip
    if not isinstance(rows, str):
        rows = out["meta"]["rows"]  # an iterator is consumed once
    ec.read_registers(R.PNOW, 2)  # link probe: a timeout here propagates (nothing answers)
    state: dict[str, Any] = {"records": (), "classes": {}, "lost": False}

    def run(name: str, fn: Callable[[], None]) -> None:
        if state["lost"]:
            errors.append(f"{name}: skipped (link lost)")
            return
        try:
            fn()
        except ModbusTimeout as exc:
            state["lost"] = True
            errors.append(f"{name}: no answer ({exc})")
        except (EcDriverError, LookupError, ValueError) as exc:
            errors.append(f"{name}: {type(exc).__name__}: {exc}")

    def identity() -> None:
        dev = ec.identify()
        data = dev.to_dict()
        for key in ("categories", "index_header", "errors"):
            data.pop(key, None)
        data.update(slave=ec.slave, baudrate=ec.baudrate)
        out["identity"] = data
        state["records"] = dev.categories or read_directory(ec)[1]
        out["directory"] = directory_summary(dev.index_header, state["records"])

    def support() -> None:
        out["support"] = read_support_words(ec, state["records"])
        out["accl"] = read_accl(ec, state["records"])

    def parameters() -> None:
        classes = []
        for ci in PARAMETER_CLASSES:
            try:
                block = read_class(ec, ci, records=state["records"])
            except LookupError:
                continue  # not in this unit's directory
            state["classes"][ci] = block
            classes.append({
                "class_id": ci, "class": f"0x{ci:04X}", "address": f"0x{block.address:04X}", "words": block.words,
                "items": [{"key": it.key, "id": it.item_id, "entry": it.entry, "address": f"0x{it.address:04X}",
                           "words": it.size, "hex": it.hex} for it in block.items],
            })  # fmt: skip
        system = (out["identity"] or {}).get("system_code")
        named: list[dict[str, Any]] = []
        if system is not None and not R.is_ec_family(system):
            for p in con_params.con_parameters(R.product_family(system)):
                block = state["classes"].get(p.class_id)
                item = None if block is None else block.item(p.item_id, p.entry)
                if item is not None and item.size == 2:
                    value = con_params.decode(p, item.raw)
                    named.append({"number": p.no, "name": p.name, "item": p.key, "value": value,
                                  "text": con_params.text(p, value), "raw": item.hex})  # fmt: skip
        else:
            named = [{k: prm[k] for k in ("no", "name", "key", "value", "text", "raw_hex", "found")}
                     for prm in decode_user_parameters(state["classes"])]  # fmt: skip
        out["parameters"] = {"classes": classes, "named": named}

    def positions() -> None:
        out["positions"] = _backup_rows(ec, state["records"], rows)

    def alarms() -> None:
        words = read_block(ec, R.CATEGORY_ALARM_HISTORY, records=state["records"])
        rtc = rtc_in_use(_read_ssse(ec, {}))
        history = []
        for k, code, detail, address, stamp in alarm_history_slots(words):
            history.append({"index": k, "code": code, "hex": f"0x{code:03X}", "name": alarm_name(code),
                            "detail": detail, "address": address, **alarm_time(stamp, rtc_in_use=rtc)})  # fmt: skip
        out["alarms"] = {"words": [f"{w:04X}" for w in words], "rtc_in_use": rtc, "history": history}

    def maintenance() -> None:
        system = (out["identity"] or {}).get("system_code")
        block = read_class(ec, R.CATEGORY_MAINTENANCE, records=state["records"])
        out["maintenance"] = decode_maintenance(block, rtc=(out["alarms"] or {}).get("rtc_in_use"),
                                          family=R.product_family(system))  # fmt: skip

    run("identity", identity)
    if state["records"]:
        for name, fn in (("support", support), ("parameters", parameters), ("positions", positions),
                         ("alarms", alarms), ("maintenance", maintenance)):  # fmt: skip
            run(name, fn)
    elif not state["lost"]:
        errors.append("directory: not readable; only the identity was saved")
    out["meta"]["duration_s"] = round(time.perf_counter() - started, 3)
    if path is not None:
        target = Path(path)
        out["meta"]["path"] = str(target)
        target.write_text(json.dumps(out, indent=2), encoding="utf-8")
    return out


def backup_axes(
    axes: Mapping[str, Elecylinder], directory: str | Path, *, rows: str | Iterable[int] = "used"
) -> dict[str, str]:
    """:func:`backup` of several axes into ``directory`` (created if missing), one ``<name>.json`` each (read-only;
    like the PC tool's multi-axis backup). Returns ``{name: file path}``, or ``{name: "error: ..."}`` for an axis
    that did not answer; the other axes are still saved."""
    folder = Path(directory)
    folder.mkdir(parents=True, exist_ok=True)
    chosen = rows if isinstance(rows, str) else sorted({int(n) for n in rows})
    result: dict[str, str] = {}
    for name, axis in axes.items():
        safe = "".join(c if c.isalnum() or c in "-_." else "_" for c in str(name)) or "axis"
        target = folder / f"{safe}.json"
        try:
            backup(axis, target, rows=chosen)
            result[name] = str(target)
        except (EcDriverError, OSError) as exc:
            result[name] = f"error: {type(exc).__name__}: {exc}"
    return result


_publish(globals(), "info")
