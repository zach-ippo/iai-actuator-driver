"""The Python part of ``iai_actuator_driver.check`` (where its documentation is): :class:`CheckReport`,
:class:`CheckItem` and the Python :func:`run_check`, which runs for an axis that is not a core axis (a test double)
or one whose bus was closed; that module re-binds these names.
"""

from __future__ import annotations

import datetime as _dt
import json
from collections.abc import Callable
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from .. import _backend
from .. import registers as R
from ..ec import Elecylinder
from ..errors import EcDriverError, ModbusTimeout
from ..values import DeviceInfo, Status
from . import publish as _publish

PASS, FAIL, WARN, SKIP = "PASS", "FAIL", "WARN", "SKIP"

_STROKE_TOLERANCE_MM = 0.005


@dataclass
class CheckItem:
    """One line of the self-test."""

    name: str
    result: str
    detail: str
    data: dict[str, Any] = field(default_factory=dict)


@dataclass
class CheckReport:
    """Everything the self-test found; serialised to the JSON report."""

    timestamp: str
    port: str
    slave: int
    strategy: str
    driver_version: str
    stroke: tuple[float, float] | None  # effective stroke used for the checks
    motion_requested: bool
    configured_stroke: tuple[float, float] | None = None
    ec_soft_limits_mm: tuple[float, float] | None = None
    ec_max_speed_mm_s: float | None = None  # the EC's maximum speed (MAXV item 7.4; 0x0812 only as the fallback)
    speed_cap_mm_s: float | None = None  # min(--max-speed, ec_max_speed_mm_s): what the driver allows
    baud: int | None = None
    model: str | None = None  # display name: the 0x0600 model string when the EC has one
    controller: str | None = None  # Report Slave ID in words (system, unit code, firmware)
    model_name: str | None = None
    serial_number: str | None = None
    system_code: int | None = None
    unit_code: int | None = None
    firmware: str | None = None
    items: list[CheckItem] = field(default_factory=list)
    unit_info: dict[str, Any] | None = None  # Elecylinder.unit_info(): identity, parameters, I/O, alarms, ...
    backend: str = field(default_factory=lambda: _backend.BACKEND)  # the implementation that ran it ("native")

    @property
    def passed(self) -> bool:
        """True when no item failed (WARN and SKIP do not fail the check)."""
        return all(item.result != FAIL for item in self.items)

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["result"] = PASS if self.passed else FAIL
        data["tool"] = "iai-check (unofficial)"
        return data

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2)

    def write_json(self, path: str | Path) -> Path:
        out = Path(path)
        out.write_text(self.to_json(), encoding="utf-8")
        return out


def run_check(
    ec: Elecylinder,
    *,
    motion: bool = False,
    port_label: str = "",
    step_mm: float = 5.0,
    speed_mm_s: float = 10.0,
    accel_g: float = 0.1,
    on_item: Callable[[CheckItem], None] | None = None,
) -> CheckReport:
    """Run the self-test against ``ec`` and return the report (``on_item`` sees each line as it completes)."""
    from iai_actuator_driver import __version__  # absolute: runs re-bound in check.py too

    report = CheckReport(
        timestamp=_dt.datetime.now(_dt.timezone.utc).astimezone().isoformat(timespec="seconds"),
        port=port_label,
        slave=ec.slave,
        strategy=ec.strategy.name,
        driver_version=__version__,
        stroke=ec.effective_stroke,
        configured_stroke=ec.stroke,
        motion_requested=motion,
        baud=ec.baudrate,
    )

    def add(name: str, result: str, detail: str, **data: Any) -> CheckItem:
        item = CheckItem(name, result, detail, data)
        report.items.append(item)
        if on_item is not None:
            on_item(item)
        return item

    # 1. link -----------------------------------------------------------------------------
    try:
        words = ec.read_registers(R.PNOW, 2)
        add("link", PASS, f"answered FC03 (PNOW = {R.from_i32_words(*words) / 100:.2f} mm)")
    except ModbusTimeout as exc:
        add("link", FAIL, f"no answer ({exc}); check port, baud rate, slave address, and that IA-OS is closed")
        _skip_rest(add, "no link")
        return report
    except EcDriverError as exc:
        add("link", FAIL, str(exc))
        _skip_rest(add, "no link")
        return report

    # 2-5. identity, index table, identification, soft limits ------------------------------
    info = _check_identity(ec, report, add)
    _check_index(info, add)
    _check_identification(info, add)
    _check_soft_limits(ec, report, add)
    _check_speed_limit(ec, report, add)

    # 6-12. status, alarm, history, tables, control ---------------------------------------
    st = _try(add, "status", ec.status)
    if st is not None:
        if st.emergency_stop:
            add("status", FAIL, "emergency stop is active", **st.to_dict())
        else:
            add(
                "status",
                PASS,
                f"pos {st.position_mm:.2f} {_units(ec)}, servo {'on' if st.servo_on else 'off'}, "
                f"homed {st.homed}, in position {st.in_position}, moving {st.moving}, paused {st.paused}",
                **st.to_dict(),
            )
        if st.alarm:
            add("alarm", FAIL, f"alarm {st.alarm_code:#05x} active", alarm_code=st.alarm_code)
        elif st.warning or st.alarm_code:
            add("alarm", WARN, f"warning {st.alarm_code:#05x}", alarm_code=st.alarm_code)
        else:
            add("alarm", PASS, "no alarm", alarm_code=0)
    _check_alarm_history(ec, add)
    _check_position_table(ec, add)
    block = _try(add, "direct_move_block", lambda: ec.read_registers(R.DIRECT_MOVE_BASE, R.DIRECT_MOVE_WORDS))
    if block is not None:
        add(
            "direct_move_block",
            PASS,
            f"0x9900 readable: PCMD {R.from_i32_words(block[0], block[1]) / 100:.2f} mm, "
            f"INP {R.from_u32_words(block[2], block[3]) / 100:.2f} mm, "
            f"VCMD {R.from_u32_words(block[4], block[5]) / 100:.2f} mm/s, ACMD {block[6] / 100:.2f} G, "
            f"PPOW {block[7]}, CTLF {block[8]:#06x}",
            words=block,
        )
    if st is not None:
        if st.modbus_enabled:
            add("modbus_control", PASS, "DSSE.PMSS = 1 (Modbus has command authority)")
        else:
            add("modbus_control", PASS, "DSSE.PMSS = 0 (as after power-up); the driver takes control (PMSL) "
                "at rest before the first command. STOP works either way")  # fmt: skip
        if st.safety_speed:
            add(
                "safety_speed",
                WARN,
                "DSS1 bit 14 (SFTY) is ON: safety-speed mode set on the EC (e.g. by IA-OS Teach mode). "
                "The driver does not change it; use set_safety_speed() only if you mean to",
                safety_speed=True,
            )
        else:
            add("safety_speed", PASS, "safety speed off", safety_speed=False)

    # 13. everything the EC tells about itself (read-only; never fails the check) -------------
    _check_unit_info(ec, report, add)
    _check_parameter_ranges(report, add)

    # 14-15. optional motion --------------------------------------------------------------
    if not motion:
        add("motion", SKIP, "not requested (run with --motion for a short move of up to 5 mm / deg)")
    elif st is None or not report.passed:
        add("motion", SKIP, "skipped because a read-only check failed")
    elif ec.effective_stroke is None:
        add("motion", SKIP, "stroke unknown (EC soft limits not readable): pass --stroke MIN MAX")
    else:
        _check_motion(ec, st, add, step_mm, speed_mm_s, accel_g)
    return report


def _skip_rest(add: Callable[..., CheckItem], why: str) -> None:
    for name in (
        "slave_id", "index_table", "identification", "soft_limits", "speed_limit", "status", "alarm",
        "alarm_history", "position_table", "direct_move_block", "unit_info", "parameter_ranges", "motion",
    ):  # fmt: skip
        add(name, SKIP, why)


def _check_parameter_ranges(report: CheckReport, add: Callable[..., CheckItem]) -> None:
    """The parameter values the unit holds against the driver's documented ranges (no I/O: from ``unit_info``;
    WARN at most, never FAIL). CON: values a write would refuse (:func:`iai_actuator_driver.con_params.audit`), above all a
    No.10 below the minimum positioning band (cold-start alarm 0x0A1 at the next reset, ME0342-4H p.8-22). EC: the
    present P4 (the floor for later P4 writes, ME3816-6B p.6-10)."""
    params = (report.unit_info or {}).get("parameters")
    if not params:
        add("parameter_ranges", SKIP, "parameters not read")
        return
    check = params.get("range_check")
    if check is not None:
        findings = check.get("findings") or []
        family = check.get("family") or "PCON-CB"
        fallback = "" if check.get("family") else " (the family has no range table of its own in the driver)"
        band = check.get("min_band") or {}
        band_text = f"; minimum positioning band {band['mm']:.2f} mm ({band['formula']})" if band else ""
        if findings:
            add("parameter_ranges", WARN, f"{len(findings)} value(s) outside the documented {family} ranges{fallback}: "
                + "; ".join(f["problem"] for f in findings[:3]) + band_text, **check)  # fmt: skip
        else:
            add("parameter_ranges", PASS, f"every parameter value inside the documented {family} ranges{fallback}"
                f"{band_text}", **check)  # fmt: skip
        return
    named = {p.get("no"): p for p in params.get("named") or []}
    p4 = named.get(4) or {}
    if p4.get("found"):
        add("parameter_ranges", PASS, f"EC user parameters read (categories per ME3816-6B p.6-6); P4 home position "
            f"adjustment is {p4.get('text')}: keep this report, ME3816-6B p.6-10 forbids a P4 below the value at "
            "shipping", p4_value=p4.get("value"))  # fmt: skip
    else:
        add("parameter_ranges", PASS, "EC user parameters read (categories per ME3816-6B p.6-6)")


def _try(add: Callable[..., CheckItem], name: str, fn: Callable[[], Any]) -> Any:
    try:
        return fn()
    except EcDriverError as exc:
        add(name, FAIL, f"{type(exc).__name__}: {exc}")
        return None


def _check_unit_info(ec: Elecylinder, report: CheckReport, add: Callable[..., CheckItem]) -> None:
    try:
        data = ec.unit_info()
    except (EcDriverError, LookupError, ValueError) as exc:
        add("unit_info", WARN, f"not read: {type(exc).__name__}: {exc}")
        return
    report.unit_info = data
    gaps = data.get("errors") or []
    if gaps:
        add("unit_info", WARN, f"read with {len(gaps)} gap(s): " + "; ".join(gaps[:3]), errors=gaps)
    else:
        add("unit_info", PASS, "identity, parameters, I/O, status words, alarms, maintenance and positions "
            "read (report field unit_info)")  # fmt: skip


def _check_identity(ec: Elecylinder, report: CheckReport, add: Callable[..., CheckItem]) -> DeviceInfo | None:
    info = _try(add, "slave_id", ec.identify)
    if info is None:
        return None
    report.model, report.controller, report.system_code = info.model, info.controller, info.system_code
    report.unit_code, report.firmware = info.unit_code, info.firmware
    report.model_name, report.serial_number = info.model_name, info.serial_number
    raw = info.slave_id_raw.hex(" ") if info.slave_id_raw else None
    if info.system_code is None:
        # Not answered or rejected: some firmware may not support FC11. The index table decides.
        add("slave_id", WARN, "Report Slave ID (FC11) not answered; model not read", errors=list(info.errors))
    elif info.identity_source != "FC11" and info.system_code in R.SYSTEM_CODES:
        add("slave_id", WARN, f"Report Slave ID (FC11) not answered; identity read from {info.identity_source}: "
            f"{info.controller}, firmware {info.firmware}", system_code=info.system_code, unit_code=info.unit_code,
            firmware=info.firmware, errors=list(info.errors))  # fmt: skip
    elif info.system_code is not None:
        add(
            "slave_id",
            PASS,
            f"{info.controller}, firmware {info.firmware}" + (f"; {info.actuator}" if info.actuator else ""),
            system_code=info.system_code,
            unit_code=info.unit_code,
            firmware=info.firmware,
            actuator_kind=info.actuator_kind,
            axis_number=info.axis_number,
            raw=raw,
        )
    return info


def _check_index(info: DeviceInfo | None, add: Callable[..., CheckItem]) -> None:
    if info is None or info.index_header is None or not info.categories:
        add("index_table", FAIL, "index table at 0x0000 not readable", errors=list(info.errors) if info else [])
        return
    problems = []
    for ci, expected in R.EXPECTED_CATEGORY_BASES.items():
        cat = info.category(ci)
        if cat is None:
            problems.append(f"category {ci:#06x} missing")
        elif cat.data_address != expected:
            problems.append(f"category {ci:#06x} at {cat.data_address:#06x}, driver expects {expected:#06x}")
    io = info.category(R.CATEGORY_IO_CONTROL)
    if io is not None and io.coil_base != R.EXPECTED_COIL_BASE:
        problems.append(f"coil base {io.coil_base:#06x}, driver expects {R.EXPECTED_COIL_BASE:#06x}")
    data = {"header": asdict(info.index_header), "categories": len(info.categories)}
    if problems:
        add("index_table", FAIL, "register layout differs from the driver's: " + "; ".join(problems), **data)
    else:
        add("index_table", PASS, f"{len(info.categories)} categories; layout matches the driver", **data)


def _check_identification(info: DeviceInfo | None, add: Callable[..., CheckItem]) -> None:
    if info is None:
        add("identification", SKIP, "identify() failed")
    elif info.model_name or info.serial_number:
        add(
            "identification",
            PASS,
            f"model {info.model_name or '?'}, serial {info.serial_number or '?'}",
            model_name=info.model_name,
            short_model=info.short_model,
            serial_number=info.serial_number,
        )
    else:
        add("identification", WARN, "model / serial strings at 0x0600 not available", errors=list(info.errors))


def _check_soft_limits(ec: Elecylinder, report: CheckReport, add: Callable[..., CheckItem]) -> None:
    try:
        limits = ec.read_soft_limits()
    except EcDriverError as exc:
        limits = None
        ec.soft_limits_note = f"{type(exc).__name__}: {exc}"
    report.ec_soft_limits_mm = limits
    report.stroke = ec.effective_stroke
    cfg, ec_stroke, eff = ec.stroke, ec.ec_stroke, ec.effective_stroke
    data = {
        "configured_stroke": cfg,
        "ec_soft_limits_mm": limits,
        "ec_stroke": ec_stroke,
        "effective_stroke": eff,
        "note": ec.soft_limits_note,
    }
    u = _units(ec)
    if limits is None or ec_stroke is None:
        if cfg is None:
            add("soft_limits", WARN, f"EC soft limits not available ({ec.soft_limits_note}) and no --stroke "
                "given: motion commands will be refused until a stroke is configured", **data)  # fmt: skip
        else:
            add("soft_limits", WARN, f"EC soft limits not available ({ec.soft_limits_note}); using the "
                f"configured stroke {cfg[0]:.2f}..{cfg[1]:.2f} {u} unchecked", **data)  # fmt: skip
        return
    text = f"soft limits {limits[0]:.2f}..{limits[1]:.2f} {u} (stroke {ec_stroke[0]:.2f}..{ec_stroke[1]:.2f} {u})"
    if cfg is None:
        add("soft_limits", PASS, text + "; used as the stroke", **data)
    elif eff is None:
        add("soft_limits", FAIL, text + f"; the configured stroke {cfg[0]:.2f}..{cfg[1]:.2f} {u} does not "
            "overlap it: wrong EC or wrong --stroke", **data)  # fmt: skip
    elif cfg[0] < ec_stroke[0] - _STROKE_TOLERANCE_MM or cfg[1] > ec_stroke[1] + _STROKE_TOLERANCE_MM:
        add("soft_limits", WARN, text + f"; the configured stroke {cfg[0]:.2f}..{cfg[1]:.2f} {u} exceeds it, "
            f"the driver narrows it to {eff[0]:.2f}..{eff[1]:.2f} {u}. Check --stroke for this EC", **data)  # fmt: skip
    else:
        add("soft_limits", PASS, text + f"; configured stroke {cfg[0]:.2f}..{cfg[1]:.2f} {u} lies inside", **data)


def _fmt_g(value: float | None) -> str:
    """An acceleration limit for a check line: ``"0.30"``, or ``"n/a"`` when the EC did not give it."""
    return "n/a" if value is None else f"{value:.2f}"


def _check_speed_limit(ec: Elecylinder, report: CheckReport, add: Callable[..., CheckItem]) -> None:
    try:
        speed = ec.read_speed_limit()
    except EcDriverError as exc:
        speed = None
        ec.ec_max_speed_note = f"{type(exc).__name__}: {exc}"
    report.ec_max_speed_mm_s, report.speed_cap_mm_s = speed, ec.speed_cap_mm_s
    data = {
        "ec_max_speed_mm_s": speed,
        "configured_max_speed_mm_s": ec.max_speed_mm_s,
        "speed_cap_mm_s": ec.speed_cap_mm_s,
        "note": ec.ec_max_speed_note,
    }
    u = _units(ec)
    if speed is None:
        add("speed_limit", WARN, f"unit maximum speed (MAXV item 7.4) not available "
            f"({ec.ec_max_speed_note}); speeds capped at {ec.speed_cap_mm_s:g} {u}/s only", **data)  # fmt: skip
    else:
        limits = ec.axis_limits
        accel = "" if limits is None or (limits.max_accel_g is None and limits.max_decel_g is None) else (
            f", accel / decel max {_fmt_g(limits.max_accel_g)} / {_fmt_g(limits.max_decel_g)} G")
        add("speed_limit", PASS, f"unit maximum speed {speed:.2f} {u}/s ({ec.ec_max_speed_note}){accel}; the driver "
            f"refuses speeds above {ec.speed_cap_mm_s:.2f} {u}/s and accelerations above {ec.accel_cap_g:.2f} G",
            accel_cap_g=ec.accel_cap_g, push_cap_pct=ec.push_cap_pct, **data)  # fmt: skip


def _check_alarm_history(ec: Elecylinder, add: Callable[..., CheckItem]) -> None:
    try:
        records = ec.read_alarm_history()
    except EcDriverError as exc:
        add("alarm_history", WARN, f"not readable: {type(exc).__name__}: {exc}")
        return
    alarms = [r for r in records if not r.power_up]
    newest = next(iter(records), None)
    detail = f"{len(records)} entries, {len(alarms)} alarms"
    if newest is not None:
        detail += f"; newest (#0): {newest.name}"
    add("alarm_history", PASS, detail, entries=[r.to_dict() for r in records])


def _check_position_table(ec: Elecylinder, add: Callable[..., CheckItem]) -> None:
    try:
        entries = [ec.read_position_table(n) for n in range(R.POSITION_ACTIVE_ROWS)]
    except EcDriverError as exc:
        add("position_table", FAIL, f"{type(exc).__name__}: {exc}")
        return
    u = _units(ec)
    text = ", ".join(f"#{n} {e.position_mm:.2f} {u} @ {e.speed_mm_s:.2f} {u}/s" for n, e in enumerate(entries))
    data = {"entries": [asdict(e) for e in entries]}
    eff = ec.effective_stroke
    if eff is None:
        add("position_table", WARN, text + " (stroke unknown: not checked)", **data)
    elif all(eff[0] <= e.position_mm <= eff[1] for e in entries):
        add("position_table", PASS, text, **data)
    else:
        add("position_table", WARN, text + f" (outside the stroke {eff[0]:.2f}..{eff[1]:.2f} {u})", **data)


def _units(ec: Elecylinder) -> str:
    """The axis's position unit (mm, or deg on a rotary axis); mm when it cannot be read."""
    try:
        return ec.units
    except (EcDriverError, OSError):
        return "mm"


def _check_motion(
    ec: Elecylinder,
    st: Status,
    add: Callable[..., CheckItem],
    step_mm: float,
    speed_mm_s: float,
    accel_g: float,
) -> None:
    if st.alarm or not st.homed:
        add("motion", FAIL, "axis has an alarm" if st.alarm else "axis is not homed; run 'home' first")
        return
    if st.paused:
        if st.modbus_enabled:
            add("motion", FAIL, "axis is paused (STP on); resume it first")
        else:
            add("motion", FAIL, "unit is in PIO mode (PMSS 0): the PIO holds the pause (STP on); "
                                "take PC control or release *STP on the PIO connector first")
        return
    eff = ec.effective_stroke
    assert eff is not None
    lo, hi = eff
    u = _units(ec)
    start = st.position_mm
    # The step adapts to the unit: at most a quarter of its stroke (a short gripper or a 4 mm unit too), at least
    # 0.1, toward the side with more room.
    step = round(max(0.1, min(step_mm, 0.25 * (hi - lo))), 2)
    target = start + step if hi - start >= start - lo else start - step
    if not lo <= target <= hi or not lo <= start <= hi:
        add("motion", FAIL, f"no room for a {step} {u} move inside {lo:.2f}..{hi:.2f} {u} from {start:.2f} {u}")
        return
    enabled_here = servo_here = False
    try:
        if not st.modbus_enabled:
            ec.enable_modbus_control()
            enabled_here = True
        if not st.servo_on:
            ec.servo(True)
            servo_here = True
        for name, goal in (("motion_out", target), ("motion_back", start)):
            begin = ec.status().position_mm
            # move_to returns only once PNOW has stopped changing (PEND / MEND rise inside the band
            # before standstill on the EC), so reached_mm is the settled position.
            reached = ec.move_to(goal, speed_mm_s=speed_mm_s, accel_g=accel_g, band_mm=0.1)
            assert reached is not None
            settled = round(reached.position_mm, 2)
            add(
                name,
                PASS,
                f"{begin:.2f} -> {settled:.2f} {u} settled (target {goal:.2f}, off by "
                f"{abs(settled - goal):.2f} {u}) at {speed_mm_s} {u}/s",
                target_mm=goal,
                reached_mm=settled,
                error_mm=round(abs(settled - goal), 2),
                settled=True,
                settle_s=ec.settle_s,
                target_reached=reached.target_reached,
            )
    except EcDriverError as exc:
        notes = "; ".join(getattr(exc, "__notes__", []))
        add("motion", FAIL, f"{type(exc).__name__}: {exc}" + (f" ({notes})" if notes else ""))
    finally:
        try:
            if servo_here:
                ec.servo(False)
            if enabled_here:
                ec.disable_modbus_control()
        except EcDriverError as exc:
            add("motion_restore", WARN, f"could not restore servo/PMSL state: {exc}")


_publish(globals(), "check")
