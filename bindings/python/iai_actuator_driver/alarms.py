"""Controller alarm names, levels and groups (ALMC 0x9002, alarm history 0x0500) for ELECYLINDER.

Evidence (README "Evidence levels"): VERIFIED = confirmed on hardware, ON-WIRE = seen on the serial line,
DOC = public IAI manuals, UNVERIFIED = expected behaviour, not yet confirmed on hardware.

Names (:func:`alarm_name`): the driver's curated names (:data:`iai_actuator_driver.registers.ALARM_NAMES`), else
:data:`CONTROLLER_ALARM_NAMES`, IAI's English controller alarm names as IA-OS shows them for the controller
range 0x040-0xFFF (plus 0 = no alarm). Which of these codes an EC can actually raise is not documented.

Levels (:func:`alarm_level`, :func:`alarm_is_fault`, :func:`alarm_info`), UNVERIFIED unless marked:

* **message** = a warning: the axis keeps running; it never blocks motion and never lights the red alarm lamp.
  Only the codes of :data:`MESSAGE_CODES`, the named codes of 0x011-0x07F (e.g. maintenance warnings 0x048 /
  0x04A / 0x04E / 0x04F, deviation warning 0x047, collision warning 0x049). 0x069 was seen on a PCON-CB (VERIFIED:
  raised at power-up, cleared by an alarm reset; the warning bit DSS1 b9 stays set after the reset); that the axis
  keeps running with one active is UNVERIFIED.
* **operation cancelled**: the EC decelerates, stops and switches the servo off (DSS1 b10 ALMH); an alarm reset
  clears it. The 0x080-0x0FF codes 080, 082-08A, 090-096, 0A2-0A5, 0A7, 0AB-0AE, 0B5, 0B6, 0BA, 0BE-0C2, 0C5,
  0CF, 0D6, 0D8-0DC, 0DE, 0DF, 0E1, 0ED, 0EF. VERIFIED for 0x090 (software reset sent with the servo on: ALMH,
  servo off; an alarm reset cleared it).
* **cold start**: every other code of 0x080-0x0FF (and 0x001-0x010). The same stop, but only a software reset
  (servo off first) or a 24 V power cycle clears it, and an incremental unit then needs a home return.
* Device-dependent: **0x0EE** "ABS error (2)" is operation cancelled on an absolute unit (VERIFIED on the EC-GRBP8M
  gripper, WA battery-less absolute: ALMH, servo off; an alarm reset and then a home return cleared it) and cold
  start on an incremental unit (UNVERIFIED). The driver tells the two apart by the class-1 support bits SUPT
  b13 ABSE / SUP2 b18 MABS (:func:`absolute_encoder_from_support`); unknown, it names both. **0x0CE / 0x0D2**
  (supply voltage) count as cold start on every unit: conservative, since a software reset or a power cycle
  also clears an operation-cancelled alarm.
* **unknown**: every other code, including a code of 0x011-0x07F that is not in :data:`MESSAGE_CODES`. It
  counts as a fault unless the warning bit DSS1 b9 ALML is set (fail safe).
* CON-family units (PCON / ACON / DCON / SCON, by FC11 system code or family name): 0x02C / 0x02D / 0x02E are
  message level there (:data:`CON_MESSAGE_CODES`; DOC ME0342-4H (PCON-CB) p.8-14, ME0343-3B (ACON / DCON-CB)
  p.8-14, ME0340-9F (SCON-CB) p.8-11). On an EC (no family given) they keep the unknown level. 0x0F1 (fieldbus
  link) is operation cancelled (DOC ME0342-4H p.8-21, ME0343-3B p.8-20, ME0340-9F p.8-17). 0x0CE / 0x0D2 / 0x0EE
  keep the conservative level above on every unit; :func:`alarm_info` says per family whether an alarm reset is
  enough (PCON / ACON-CB) or a cold start is needed (SCON-CB).

Groups (:func:`alarm_group`): the EC alarm groups A-E (DOC ME3816-6B p.8-19), "F" (data setting error: a grouping
the PC tool shows, not in ME3816-6B), "Warning" (maintenance warnings 1-4, ME3816-6B p.8-19, 8-26 to 8-28) and "Other"
(UNVERIFIED; the PC tool's alarm window showed 0x0EE as group D). The remedies are the driver's own short wording with the
manual page they rest on (IAI's cause / remedy texts are not shipped).

Time stamps (:func:`alarm_time`): the history time (0x0504-0x0505) is seconds since 2000-01-01 00:00:00 when the
unit's calendar (RTC) is in use (SSSE b8 = 1; DOC ME0162-12A p.4-10), else the time since power-on (milliseconds on
the EC: VERIFIED; the manual says seconds on p.4-10 and milliseconds on p.5-13 for the CON units).

The names, levels, groups, remedies and their rules are the core's (``core/src/alarms.cpp``, ``alarm_tables.cpp``),
reached through ``iad_call_json`` (:mod:`._abi_tables`); the tables below are read from the core once, at import. The
level constants stay Python. Rules, evidence levels and manual pages: above. ``tests/test_tables_conformance.py``
compares the core's answers on every code with the tables of 1.0 (``tests/tables_snapshot.json``).
"""

from __future__ import annotations

import datetime as _dt
from collections.abc import Mapping, Sequence
from functools import lru_cache
from typing import Any, Final

from ._native import _abi_tables as _t

__all__ = [
    "CONTROLLER_ALARM_NAMES",
    "CON_ALARM_NAMES",
    "MESSAGE_CODES",
    "CON_MESSAGE_CODES",
    "CON_SYSTEM_CODES",
    "RTC_EPOCH",
    "alarm_name",
    "alarm_family",
    "alarm_time",
    "rtc_datetime",
    "WARNING_CLEARING",
    "alarm_level",
    "alarm_group",
    "alarm_info",
    "alarm_is_fault",
    "absolute_encoder_from_support",
    "alarm_history_slots",
    "describe_alarm_detail",
    "LEVEL_NONE",
    "LEVEL_MESSAGE",
    "LEVEL_OPERATION_CANCELLED",
    "LEVEL_COLD_START",
    "LEVEL_UNKNOWN",
    "LEVEL_TEXT",
    "FAULT_LEVELS",
    "ALARM_GROUPS",
    "GROUP_NAMES",
    "HOME_NEEDED",
]

LEVEL_NONE: Final = "none"  # 0x000 / the 0x0FFF power-up marker
LEVEL_MESSAGE: Final = "message"  # warning: the axis keeps running
LEVEL_OPERATION_CANCELLED: Final = "operation_cancelled"  # stop + servo off; alarm reset clears it
LEVEL_COLD_START: Final = "cold_start"  # stop + servo off; software reset / power cycle
LEVEL_UNKNOWN: Final = "unknown"  # no known level: a fault unless ALML is set (fail safe)

RTC_EPOCH: Final = _dt.datetime(2000, 1, 1, 0, 0, 0)
"""Calendar time origin of the alarm-history / TIMN time stamps when the unit's RTC is in use (ME0162-12A p.4-10);
the unit's own local time, no time zone (conformance: the core's own epoch)."""

# The tables are read from the core at the first use of one of them (native.py imports this module while the library
# is being loaded, so nothing here calls the core at import).
_TABLE_NAMES: Final = frozenset({
    "LEVEL_TEXT", "FAULT_LEVELS", "MESSAGE_CODES", "CON_MESSAGE_CODES", "CON_ALARM_NAMES", "CON_SYSTEM_CODES",
    "ALARM_GROUPS", "GROUP_NAMES", "WARNING_CLEARING", "HOME_NEEDED", "CONTROLLER_ALARM_NAMES", "IAOS_ALARM_NAMES",
})  # fmt: skip


def _tables() -> dict[str, Any]:
    """The alarm tables from the core: CONTROLLER_ALARM_NAMES (IAI's English controller alarm names, as IA-OS shows
    them), the level rules as code sets, the groups, the clearing and home-return texts; IAOS_ALARM_NAMES is the old
    name of CONTROLLER_ALARM_NAMES, kept so existing imports keep working."""
    t = _t.call("alarms.tables")
    names = dict(t["controller_names"])
    return {
        "LEVEL_TEXT": dict(t["level_text"]), "FAULT_LEVELS": frozenset(t["fault_levels"]),
        "MESSAGE_CODES": frozenset(t["message_codes"]), "CON_MESSAGE_CODES": frozenset(t["con_message_codes"]),
        "CON_ALARM_NAMES": dict(t["con_names"]), "CON_SYSTEM_CODES": frozenset(t["con_system_codes"]),
        "ALARM_GROUPS": dict(t["groups"]), "GROUP_NAMES": dict(t["group_names"]),
        "WARNING_CLEARING": dict(t["warning_clearing"]), "HOME_NEEDED": dict(t["home_needed"]),
        "CONTROLLER_ALARM_NAMES": names, "IAOS_ALARM_NAMES": names,
    }  # fmt: skip


def __getattr__(name: str) -> Any:
    if name in _TABLE_NAMES:
        globals().update(_tables())
        return globals()[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__() -> list[str]:
    return sorted(set(globals()) | _TABLE_NAMES)


@lru_cache(maxsize=8192)
def _call(function: str, *args: tuple[str, Any]) -> Any:
    """One call of the core, cached: these functions only read the core's tables (the result is never mutated)."""
    return _t.call(function, dict(args))


def alarm_name(code: int) -> str:
    """Name of controller alarm ``code``: the driver's curated name, else IAI's (:data:`CONTROLLER_ALARM_NAMES`,
    then the CON-only message codes :data:`CON_ALARM_NAMES`), else ``"alarm 0x..."``."""
    return _call("alarms.alarm_name", ("code", code))  # type: ignore[no-any-return]


def alarm_family(system_code: int | None = None, family: str | None = None) -> str | None:
    """The CON family an alarm is read for: ``"PCON"``, ``"ACON/DCON"`` or ``"SCON"``, else ``None`` (an EC, or a
    unit whose family is not known). ``family`` (a product-family name such as ``"PCON"``, ``"ACON"``, ``"DCON"``)
    wins over ``system_code`` (FC11)."""
    return _call(  # type: ignore[no-any-return]
        "alarms.alarm_family", ("system_code", system_code), ("family", str(family) if family else None)
    )


def rtc_datetime(seconds: int) -> _dt.datetime:
    """The calendar time of an RTC time stamp: ``seconds`` since 2000-01-01 00:00:00 (ME0162-12A p.4-10), as a naive
    datetime in the unit's own local time."""
    return _dt.datetime.fromisoformat(_call("alarms.rtc_datetime", ("seconds", int(seconds))))


def alarm_time(time_raw: int | None, *, rtc_in_use: bool | None = None) -> dict[str, Any]:
    """Decode an alarm-history time stamp (0x0504-0x0505, ALT0; no I/O): ``time_raw``, ``clock`` (``"rtc"`` with
    ``occurred_at`` when ``rtc_in_use``, else ``"power_on"`` with ``time_after_power_on`` read as milliseconds),
    ``None`` times for ``time_raw`` ``None`` (the unit stored FFFFFFFF)."""
    return dict(_call("alarms.alarm_time", ("time_raw", time_raw), ("rtc_in_use", rtc_in_use)))


def absolute_encoder_from_support(support: Mapping[str, int] | None) -> bool | None:
    """Whether the unit has an absolute encoder, from its class-1 support words (``{"SUPT": ..., "SUP2": ...}``,
    :func:`iai_actuator_driver.info.read_support_words`). No I/O. True when SUPT b13 ABSE or SUP2 b18 MABS is set,
    False when SUPT was read and neither bit is set, ``None`` when it cannot be told."""
    if not support:
        return None
    words = {str(k): (None if v is None else int(v)) for k, v in support.items()}
    return _t.call("alarms.absolute_encoder_from_support", {"support": words})  # type: ignore[no-any-return]


def alarm_level(
    code: int, *, absolute_encoder: bool | None = None, system_code: int | None = 0xEC, family: str | None = None
) -> str:
    """Level of controller alarm ``code`` (``"none"``, ``"message"``, ``"operation_cancelled"``, ``"cold_start"``,
    ``"unknown"``). ``absolute_encoder`` only matters for 0x0EE; ``system_code`` (FC11) / ``family``
    (:func:`alarm_family`) only for the CON-only message codes :data:`CON_MESSAGE_CODES`."""
    return _call(  # type: ignore[no-any-return]
        "alarms.alarm_level", ("code", code), ("absolute_encoder", absolute_encoder), ("system_code", system_code),
        ("family", str(family) if family else None),
    )


def alarm_is_fault(
    code: int,
    *,
    alarm_bit: bool = False,
    warning_bit: bool = False,
    system_code: int | None = 0xEC,
    family: str | None = None,
) -> bool:
    """The driver's fault rule: DSS1 b10 ALMH (``alarm_bit``), or a code of level operation cancelled / cold start; a
    code of unknown level is a fault unless the warning bit ALML (``warning_bit``) is set (fail safe)."""
    return _call(  # type: ignore[no-any-return]
        "alarms.alarm_is_fault", ("code", code), ("alarm_bit", bool(alarm_bit)), ("warning_bit", bool(warning_bit)),
        ("system_code", system_code), ("family", str(family) if family else None),
    )


def alarm_group(code: int) -> str:
    """EC alarm group letter (A-F), ``"Warning"`` (maintenance) or ``"Other"`` (UNVERIFIED)."""
    return _call("alarms.alarm_group", ("code", code))  # type: ignore[no-any-return]


def alarm_history_slots(
    words: Sequence[int], *, include_empty: bool = False
) -> list[tuple[int, int, int, int, int | None]]:
    """Decode alarm-history words (0x0500, 6 words per slot): ``(index, code, detail, address, time_ms)`` per slot,
    index 0 the newest, ``time_ms`` ``None`` for FFFFFFFF; the list ends at the first empty slot unless
    ``include_empty``. No I/O."""
    rows = _t.call("alarms.alarm_history_slots", {"words": [int(w) for w in words], "include_empty": include_empty})
    return [tuple(row) for row in rows]  # type: ignore[misc]


def describe_alarm_detail(code: int, detail: int | None, address: int | None) -> str:
    """Decode an alarm's detail / address words where their meaning is known ("" otherwise; UNVERIFIED)."""
    return _call(  # type: ignore[no-any-return]
        "alarms.describe_alarm_detail", ("code", code), ("detail", detail), ("address", address)
    )


def alarm_info(
    code: int,
    *,
    absolute_encoder: bool | None = None,
    detail: int | None = None,
    address: int | None = None,
    system_code: int | None = 0xEC,
    family: str | None = None,
) -> dict[str, Any]:
    """Everything the driver knows about ``code`` as a JSON-friendly dict (no I/O): ``code``, ``hex``, ``name``,
    ``level``, ``level_text``, ``group`` / ``group_name``, ``fault``, ``warning``, ``reset``, ``home_needed``,
    ``remedy``, ``detail_text`` and ``family`` (the CON family it was read for, else ``None``)."""
    return dict(
        _call(
            "alarms.alarm_info", ("code", code), ("absolute_encoder", absolute_encoder), ("detail", detail),
            ("address", address), ("system_code", system_code), ("family", str(family) if family else None),
        )
    )
