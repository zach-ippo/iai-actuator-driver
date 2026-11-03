"""The value types of ``iai_actuator_driver.con_params``: :class:`ConParam`,
:class:`ConUnitLimits`, :class:`MinBand` and :class:`ConCheck`.

The table and the rules are the core's; ``iai_actuator_driver.con_params`` (with the kinds, the evidence and the manual
pages) builds these very classes from its answers.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from . import publish


@dataclass(frozen=True)
class ConParam:
    """One numbered CON parameter: ``no``, ``name``, ``symbol``, ``class_id`` / ``item_id`` (/ ``entry``), ``kind``
    (encoding, see the module text), ``scale`` / ``signed`` / ``bits``, ``unit``, value range ``lo`` .. ``hi`` (the
    PCON-CB range; :func:`for_family` for the others), ``choices`` (value -> label), ``dangerous`` (written with
    a warning), ``writable`` and ``note`` (a documented caution shown with the parameter)."""

    no: int
    name: str
    symbol: str
    class_id: int
    item_id: int
    kind: str
    entry: int = 0
    scale: float = 1.0
    signed: bool = False
    bits: tuple[int, ...] = ()
    unit: str = ""
    lo: float | None = None
    hi: float | None = None
    choices: dict[int, str] = field(default_factory=dict)
    dangerous: bool = False
    writable: bool = True
    note: str = ""

    @property
    def key(self) -> str:
        cls = f"{self.class_id:X}" if self.class_id > 9 else str(self.class_id)
        return f"{cls}.{self.item_id}" + (f"[{self.entry}]" if self.entry else "")


@dataclass(frozen=True)
class ConUnitLimits:
    """What a CON unit says about itself, for :func:`check_value` (``None`` = not known).

    ``max_speed`` MAXV 7.4 and ``max_push_speed`` MXPH 7.8 (units/s), ``max_accel`` MAXA 7.5 (G), ``lead_mm``
    LEAD 7.3 (= No.77), ``encoder_pulses`` EPLS 9.12 (item meaning UNVERIFIED: it reads 8192 on the RCP6, the
    encoder count the manual gives for the RCP6), ``min_band_mm`` MINI 3.67 when the unit enforces it (SUP2 CINP
    and ATYP RINP set), ``rotary`` (ATYP 7.7 b8)."""

    max_speed: float | None = None
    max_accel: float | None = None
    max_push_speed: float | None = None
    lead_mm: float | None = None
    encoder_pulses: int | None = None
    min_band_mm: float | None = None
    rotary: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "max_speed": self.max_speed, "max_accel": self.max_accel, "max_push_speed": self.max_push_speed,
            "lead_mm": self.lead_mm, "encoder_pulses": self.encoder_pulses, "min_band_mm": self.min_band_mm,
            "rotary": self.rotary,
        }  # fmt: skip


@dataclass(frozen=True)
class MinBand:
    """A minimum positioning band: ``mm``, the ``formula`` it came from, ``doc`` (manual page) and ``certain``
    (False when the encoder count was not known and 800 pulses were assumed: a warning, not a refusal)."""

    mm: float
    formula: str
    doc: str
    certain: bool = True


@dataclass(frozen=True)
class ConCheck:
    """The result of :func:`check_value`: the parsed ``value``, ``warnings`` (written anyway), the ``family`` the
    range came from (``None`` = PCON-CB range used for an unknown family) and the range ``lo`` .. ``hi`` applied."""

    value: int | float | str
    warnings: tuple[str, ...]
    family: str | None
    lo: float | None
    hi: float | None


publish(globals(), "con_params")
