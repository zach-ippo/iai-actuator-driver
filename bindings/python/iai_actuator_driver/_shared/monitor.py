"""The Python part of ``iai_actuator_driver.monitor``: the poll timing constants, the
``completion="fast"`` rule (:func:`fast_done`) and the published sample (:class:`MonitorSample`). No I/O.

``iai_actuator_driver.monitor`` (where the protocol notes and the evidence are) imports these names from here.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import TYPE_CHECKING

from . import publish

if TYPE_CHECKING:
    from ..values import Status

LEAN_STATUS_WORDS = 8  # 0x9000..0x9007: PNOW (2), ALMC, DIPM, DOPM, DSS1, DSS2, DSSE; the smallest range with all four
FRESH_S = 0.05  # a monitor status younger than this replaces the pre-move status read
POLL_GAP_S = 0.002  # pause between two monitor polls, with the port lock free (other threads and axes get the port)
STALL_S = 0.25  # a waiter falls back to its own read when the monitor has published nothing this long
BACKOFF_MIN_S = 0.01  # after a transient error (timeout / CRC / frame / exception response) ...
BACKOFF_MAX_S = 0.5  # ... doubling up to this
LINK_CHECK_S = 1.0  # recovery.check_link (one full status read) at most this often, while the axis has a session
SILENT_AFTER = 3  # failed polls in a row that count as a lost link ("unit_silent", or "port_vanished" when unlisted)
_POS_EPS = 1e-6  # mm; positions are 0.01 mm counts, this only absorbs float noise


def fast_done(st: Status, target_mm: float | None = None, band_mm: float | None = None) -> bool:
    """The ``completion="fast"`` rule on one sample: PEND set, ``|position - target| <= band`` (``band`` 0.1 when
    ``None``; no position check without a target), DSSE MOVE clear, not homing and not paused."""
    if not st.in_position or st.move_bit or st.homing or st.paused:
        return False
    if target_mm is None:
        return True
    band = 0.1 if band_mm is None else band_mm
    return abs(st.position_mm - target_mm) <= band + _POS_EPS


@dataclass(frozen=True)
class MonitorSample:
    """One published poll: the decoded lean ``status``, its sequence number and the ``time.perf_counter()`` times just
    before the request went out (``t_start``) and after the answer was decoded (``t``)."""

    status: Status
    seq: int
    t_start: float
    t: float

    @property
    def age_s(self) -> float:
        return time.perf_counter() - self.t


publish(globals(), "monitor")
