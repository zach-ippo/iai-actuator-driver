"""The JSON outputs of the C ABI keep their fields: core/abi/json_keys.json against what the library writes now.

iai_actuator_driver.h ("Stability"): within an ABI major a JSON field is only added, never removed, renamed or retyped
(an integer stays an integer, a float a float; null means unknown). core/abi/json_keys.json is the committed list: for
every JSON output of the stable header (each ``*_json`` function, iad_axis_restart_signs, iad_bus_set_baud_rate_all,
every ``iad_call_json`` function, every ``iad_axis_info_call_json`` reader, the error JSON and the kept results of the
operations below) the path of each field and its JSON types, sampled by one fixed scenario on the native simulator.
The test fails when a listed field is missing or has another type now; a new field passes.

    python tests/test_abi_json_keys.py --update      rewrite core/abi/json_keys.json from the scenario (after a
                                                     deliberate change; review the diff: within a major only additions)

Paths: ``$`` the output, ``.name`` a field, ``[]`` the elements of an array, ``{}`` the values of a map (an object
keyed by data: numbers, hex addresses, "port:slave" axis names). Types: object, array, string, int, float, bool, null.
"""

from __future__ import annotations

import ctypes
import json
import math
import os
import re
import sys
import tempfile
import unittest
from collections.abc import Callable
from pathlib import Path
from typing import Any

from iai_actuator_driver import native

ROOT = Path(__file__).resolve().parent.parent
KEYS_FILE = ROOT.parents[1] / "core" / "abi" / "json_keys.json"
NAN = float("nan")

# A map: an object keyed by data, not by field names (its values are merged under "{}").
_MAP_KEY = re.compile(r"^(-?\d+|0x[0-9A-Fa-f]+|[^:\s]+:\d+)$")

# Arguments of the iad_call_json functions (one sample each; "functions" lists them).
CALL_ARGS: dict[str, Any] = {
    "alarms.tables": {},
    "alarms.alarm_name": {"code": 0x0E8},
    "alarms.alarm_family": {"system_code": 174, "family": "PCON"},
    "alarms.alarm_level": {"code": 0x0E8},
    "alarms.alarm_is_fault": {"code": 0x0E8, "alarm_bit": True},
    "alarms.alarm_group": {"code": 0x0E8},
    "alarms.alarm_info": {"code": 0x0E8, "family": "PCON"},
    "alarms.describe_alarm_detail": {"code": 0x0E8, "detail": 1, "address": 0},
    "alarms.absolute_encoder_from_support": {"support": None},
    "alarms.alarm_history_slots": {"words": [0] * 64, "include_empty": True},
    "alarms.alarm_time": {"time_raw": 1000, "rtc_in_use": False},
    "alarms.rtc_datetime": {"seconds": 1000},
    "con_params.tables": {},
    "con_params.con_family": {"family": "PCON"},
    "con_params.con_parameters": {"family": "PCON"},
    "con_params.category": {"no": 7, "family": "PCON"},
    "con_params.factory_default": {"no": 7, "family": "PCON"},
    "con_params.con_unit_limits": {"classes": [], "support": None},
    "con_params.min_positioning_band": {"family": "PCON", "lead_mm": 10.0, "encoder_pulses": 800},
    "info.bit_tables": {},
    "info.decode_bits": {"word": 5, "table": "DSS1"},
    "info.position_mode": {"code": 0, "model": "EC"},
    "info.rtc_in_use": {"ssse": 0x100},
    "status.decode": {"words": [0] * 8},
    "info.product_line": {"family": "PCON"},
    "info.ec_parameter_table": {},
    "info.input_ranges": {"line": "EC"},
}
# Functions whose arguments come from another function's output (filled in by the scenario).
CALL_ARGS_FROM: dict[str, Callable[[dict[str, Any]], Any]] = {
    "con_params.for_family": lambda s: {"p": s["p"], "family": "PCON"},
    "con_params.check_value": lambda s: {"p": s["p"], "value": s["p_value"], "family": "PCON"},
    "con_params.audit": lambda s: {"family": "PCON", "values": [[s["p"]["no"], s["p_value"]]]},
    "con_params.parameter_table": lambda s: {"family": "PCON"},
    "con_params.decode": lambda s: {"p": s["p"], "raw": 100},
    "con_params.encode": lambda s: {"p": s["p"], "raw": 0, "value": s["p_value"]},
    "con_params.text": lambda s: {"p": s["p"], "value": s["p_value"]},
    "con_params.parse": lambda s: {"p": s["p"], "value": s["p_value"]},
    "info.io_bit_tables": lambda s: {"mode": s["position_mode"]},
    "info.decode_maintenance": lambda s: {"block": s["block"]},
    "info.decode_alarm_history": lambda s: {"record": s["record"], "words": [0] * 64},
}


# ====================================================================== the shape of a JSON value


def _type(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "float"
    if isinstance(value, str):
        return "string"
    if isinstance(value, list):
        return "array"
    return "object"


def shape(value: Any, path: str = "$", out: dict[str, set[str]] | None = None) -> dict[str, set[str]]:
    """{path: {JSON types}} of every field of `value` (arrays and maps merge their elements)."""
    out = {} if out is None else out
    out.setdefault(path, set()).add(_type(value))
    if isinstance(value, list):
        for item in value:
            shape(item, f"{path}[]", out)
    elif isinstance(value, dict):
        if value and all(_MAP_KEY.match(k) for k in value):
            for item in value.values():
                shape(item, f"{path}{{}}", out)
        else:
            for k, item in value.items():
                shape(item, f"{path}.{k}", out)
    return out


def _flat(sample: dict[str, set[str]]) -> dict[str, str]:
    return {p: "|".join(sorted(t)) for p, t in sorted(sample.items())}


def differences(committed: dict[str, dict[str, str]], now: dict[str, dict[str, str]]) -> list[str]:
    """Fields of the committed list that are gone or retyped now (null on either side is no retype; a field under an
    array or map that is empty now cannot be seen and is not checked)."""
    problems = []
    for label, fields in committed.items():
        if label not in now:
            problems.append(f"{label}: not sampled any more")
            continue
        seen = now[label]
        for path, types in fields.items():
            if path not in seen:
                cut = max(path.rfind("[]"), path.rfind("{}"))
                element = path[: cut + 2] if cut >= 0 else None
                if element == path and path[:-2] in seen:
                    continue  # the array or map is empty now
                if element is not None and element != path and element not in seen:
                    continue  # the array or map holding it has no element now: nothing to compare
                problems.append(f"{label}: {path} ({types}) is missing (removed or renamed)")
                continue
            had = set(types.split("|")) - {"null"}
            has = set(seen[path].split("|")) - {"null"}
            if had and has and not has <= had:
                problems.append(f"{label}: {path} was {types}, now {seen[path]}")
    return problems


# ====================================================================== the scenario


class _Scenario:
    """One fixed tour of the stable header's JSON outputs on the simulator (no port is opened)."""

    def __init__(self, tmp: str) -> None:
        self.lib = native.load()
        self.tmp = tmp
        self.samples: dict[str, dict[str, set[str]]] = {}
        self.cleanup: list[Callable[[], Any]] = []
        self.errors: list[str] = []

    # ---------------------------------------------------------------- plumbing

    def json(self, label: str, name: str, *args: Any) -> Any:
        for attempt in range(3):
            try:
                value = native._read_json(self.lib, name, *args)
                break
            except RuntimeError as exc:  # a reader that reads again when its text did not fit can grow meanwhile
                if "kept growing" not in str(exc) or attempt == 2:
                    self.errors.append(f"{label}: {type(exc).__name__}: {exc}")
                    return None
            except Exception as exc:  # noqa: BLE001 - every output is tried; the failures are reported together
                self.errors.append(f"{label}: {type(exc).__name__}: {exc}")
                return None
        self.take(label, value)
        return value

    def take(self, label: str, value: Any) -> None:
        shape(value, "$", self.samples.setdefault(label, {}))

    def check(self, name: str, *args: Any) -> None:
        native._check(self.lib, name, *args)

    def error(self, label: str, name: str, *args: Any) -> None:
        code, details = native._invoke(self.lib, name, *args)
        assert code != native.IAD_OK and details is not None, f"{name} did not fail"
        self.take(label, details)

    def handle(self, name: str, *args: Any, close: str | None = None) -> int:
        out = ctypes.c_void_p()
        self.check(name, *args, ctypes.byref(out))
        value = int(out.value or 0)
        if close is not None:
            self.cleanup.append(lambda: native._check(self.lib, close, value))
        return value

    def close(self) -> None:
        for step in reversed(self.cleanup):
            try:
                step()
            except Exception:  # noqa: BLE001 - best effort: the samples are taken
                pass

    # ---------------------------------------------------------------- the tour

    def run(self) -> dict[str, dict[str, str]]:
        try:
            self.library()
            self.calls()
            self.axis()
            self.cell()
            self.gateway()
        finally:
            self.close()
        assert not self.errors, "the scenario failed:\n  " + "\n  ".join(self.errors)
        return {label: _flat(sample) for label, sample in sorted(self.samples.items())}

    def library(self) -> None:
        self.json("iad_constants_json", "iad_constants_json")
        self.json("iad_line_rules_json", "iad_line_rules_json")
        ports = [{"port": "COM7", "description": "USB Serial Port", "manufacturer": "FTDI",
                  "hardware_id": "USB VID:PID=0403:6015", "vid": 1027, "pid": 24597}]
        self.check("iad_testing_set_port_list", json.dumps(ports).encode())
        try:
            self.json("iad_list_ports_json", "iad_list_ports_json")
        finally:
            self.check("iad_testing_set_port_list", None)
        self.error("iad_last_error_json(invalid_argument)", "iad_axis_open_sim", b"no-such-unit", None, None, None,
                   ctypes.byref(ctypes.c_void_p()))

    def calls(self) -> None:
        names = self.json("iad_call_json(functions)", "iad_call_json", b"functions", None)
        state: dict[str, Any] = {}
        params = native._read_json(self.lib, "iad_call_json", b"con_params.con_parameters", b'{"family": "PCON"}')
        state["p"] = next(p for p in params if p.get("kind") == "scaled" and p.get("writable"))
        state["p_value"] = state["p"].get("default") if state["p"].get("default") is not None else 1.0
        state["position_mode"] = native._read_json(self.lib, "iad_call_json", b"info.position_mode",
                                                   json.dumps(CALL_ARGS["info.position_mode"]).encode())
        state["block"] = {"class_id": 4, "address": 0x4000, "words": 8, "member_size": 2, "entries": 4,
                          "items": [{"entry": 0, "index": 0, "address": 0x4000, "size": 2, "raw": 120}]}
        state["record"] = {"category_id": 3, "model_code": 1, "member_count": 4, "list_count": 16,
                           "data_address": 0x4100, "id_list_address": 0, "coil_base": 0, "member_size": 4}
        missing = [n for n in names if n != "functions" and n not in CALL_ARGS and n not in CALL_ARGS_FROM]
        assert not missing, f"no sample arguments for the iad_call_json functions {missing} (add them to CALL_ARGS)"
        for name in names:
            if name == "functions":
                continue
            args = CALL_ARGS[name] if name in CALL_ARGS else CALL_ARGS_FROM[name](state)
            self.json(f"iad_call_json({name})", "iad_call_json", name.encode(), json.dumps(args).encode())

    def axis(self) -> None:
        ax = native.NativeAxis.open_sim("slider", time_scale=20.0)
        self.cleanup.append(ax.close)
        h = ax._h()
        ax.connect()
        ax.servo(True)
        ax.home()
        for name in ("iad_axis_identity_json", "iad_axis_limits_json", "iad_axis_parameters_json",
                     "iad_axis_line_json", "iad_axis_read_monitor_json", "iad_axis_state_json", "iad_axis_kind_json",
                     "iad_axis_read_soft_limits_json", "iad_axis_read_axis_limits_json",
                     "iad_axis_read_support_words_json", "iad_axis_categories_json", "iad_axis_user_parameters_json",
                     "iad_axis_gateway_status_json"):
            self.json(name, name, h)
        self.json("iad_axis_read_alarm_history_json", "iad_axis_read_alarm_history_json", h, 1)
        self.json("iad_axis_parameter_model_json", "iad_axis_parameter_model_json", h, 1)
        self.json("iad_axis_push_speed_json", "iad_axis_push_speed_json", h, NAN)
        self.json("iad_axis_servo_on_timeout_json", "iad_axis_servo_on_timeout_json", h, None, 0)
        first = native._read_json(self.lib, "iad_axis_parameters_json", h)[0]
        self.json("iad_axis_read_parameter_json", "iad_axis_read_parameter_json", h, str(first["number"]).encode())
        # "note" is there only when a row has one: no row of the slider has one, so it is sampled from the rod's
        # read-only P8 (an enum; the other fields stay the slider's)
        rod = native.NativeAxis.open_sim("rod", time_scale=20.0)
        self.cleanup.append(rod.close)
        rod.connect()
        rows = native._read_json(self.lib, "iad_axis_parameters_json", rod._h())
        self.take("iad_axis_parameters_json", [{"note": row["note"]} for row in rows if "note" in row])
        p8 = native._read_json(self.lib, "iad_axis_read_parameter_json", rod._h(), b"8")
        self.take("iad_axis_read_parameter_json", {"note": p8["note"]})
        self.json("iad_axis_read_unit_info_json", "iad_axis_read_unit_info_json", h, None)
        self.json("iad_axis_backup_json", "iad_axis_backup_json", h, b"used", None)
        handles = (ctypes.c_void_p * 1)(h)
        self.json("iad_backup_axes_json", "iad_backup_axes_json", handles, 1, None, os.fsencode(self.tmp), b"used")
        # the directory readers by name, each kept for iad_axis_last_result_json
        directory = self.json("iad_axis_info_call_json(read_directory)", "iad_axis_info_call_json", h,
                              b"read_directory", None)
        class_id = directory["records"][0][0]
        for name, args in (("read_block", {"class_id": class_id, "count": 2}), ("read_class", {"class_id": class_id}),
                           ("read_support_words", {}), ("read_accl", {}), ("unit_info", {"sections": ["identity"]}),
                           ("backup", {"rows": "used"})):
            self.json(f"iad_axis_info_call_json({name})", "iad_axis_info_call_json", h, name.encode(),
                      json.dumps(args).encode())
        self.json("iad_axis_last_result_json(info_call)", "iad_axis_last_result_json", h)
        # a move handle
        o = native.MoveOptions()
        self.check("iad_move_options_init", ctypes.byref(o))
        mh = self.handle("iad_axis_start_move_to", h, 20.0, ctypes.byref(o), close="iad_move_handle_close")
        self.json("iad_move_handle_json", "iad_move_handle_json", mh)
        self.check("iad_move_handle_wait", mh, 20.0, ctypes.byref(native.Status()))
        # the status monitor, the session, a recovery, a self-test: each operation's kept result
        ax.start_monitor()
        self.json("iad_axis_monitor_info_json", "iad_axis_monitor_info_json", h)
        ax.stop_monitor() if hasattr(ax, "stop_monitor") else self.check("iad_axis_stop_monitor", h, 2.0)
        ax.track()
        self.json("iad_axis_session_json", "iad_axis_session_json", h)
        words = (ctypes.c_uint16 * 8)()
        self.json("iad_axis_restart_signs", "iad_axis_restart_signs", h, words, 8, None)
        ax.recover()
        self.json("iad_axis_last_result_json(recover)", "iad_axis_last_result_json", h)
        self.json("iad_last_result_json(recover)", "iad_last_result_json")
        ax.run_check()
        self.json("iad_axis_last_result_json(self_test)", "iad_axis_last_result_json", h)
        self.json("iad_axis_self_test_items_json", "iad_axis_self_test_items_json", h, 0)
        # a soft-limit error's details
        o2 = native.MoveOptions()
        self.check("iad_move_options_init", ctypes.byref(o2))
        self.error("iad_last_error_json(soft_limit)", "iad_axis_move_to", h, 1.0e6, ctypes.byref(o2),
                   ctypes.byref(native.Status()))
        # the bus and its transport
        bus = ax.bus()
        b = bus._h()
        for name in ("iad_bus_info_json", "iad_bus_line_json", "iad_bus_port_info_json"):
            self.json(name, name, b)
        self.json("iad_bus_identify_json", "iad_bus_identify_json", b, 1)
        self.json("iad_bus_read_device_info_json", "iad_bus_read_device_info_json", b, 1)
        self.json("iad_bus_scan_units_json", "iad_bus_scan_units_json", b, b"1", 0.2, 1, 1)
        t = self.handle("iad_bus_transport", b, close="iad_transport_release")
        self.json("iad_transport_info_json", "iad_transport_info_json", t)
        # a parameter written with its own value (the guard skips it)
        ax.write_parameter(first["number"], first["value"])
        self.json("iad_axis_last_result_json(write_parameter)", "iad_axis_last_result_json", h)
        # the line's baud change, on a unit that models its line's rate
        fast = native.NativeAxis.open_sim("slider", sim_baudrate=38400, baudrate=38400)
        self.cleanup.append(fast.close)
        fast.connect()
        bo = native.BaudOptions()
        self.check("iad_baud_options_init", ctypes.byref(bo))
        fast_bus = fast.bus()  # kept: the handle is released with the object
        self.json("iad_bus_set_baud_rate_all", "iad_bus_set_baud_rate_all", fast_bus._h(), 115200, b"1",
                  ctypes.byref(bo))

    def cell(self) -> None:
        cell = native.NativeCell.open_sim("slider,gripper", time_scale=20.0)
        self.cleanup.append(cell.close)
        c = cell._h()
        cell.servo_all(True)
        cell.home_all()
        for name in ("iad_cell_info_json", "iad_cell_lines_json", "iad_cell_gateway_status_json",
                     "iad_cell_port_errors_json"):
            self.json(name, name, c)
        self.json("iad_cell_monitor_all_json", "iad_cell_monitor_all_json", c, None)
        for what in ("identify", "limits", "monitor"):
            self.json(f"iad_cell_read_all_json({what})", "iad_cell_read_all_json", c, what.encode(), None)
        self.json("iad_cell_backup_json", "iad_cell_backup_json", c, None, os.fsencode(self.tmp), b"used")
        self.json("iad_cell_last_result_json(backup)", "iad_cell_last_result_json", c)
        names = list(cell._read_names())
        request = {"axes": {"x": names[0], "y": names[1]}, "speed": 20.0, "accel_g": 0.1,
                   "path": {"segments": [{"line": {"x": 10.0, "y": 5.0}}]}}
        self.json("iad_cell_coord_plan_json", "iad_cell_coord_plan_json", c, json.dumps(request).encode())
        self.json("iad_cell_coord_run_json", "iad_cell_coord_run_json", c, json.dumps(request).encode())
        self.json("iad_cell_last_result_json(coord_run)", "iad_cell_last_result_json", c)
        self.check("iad_cell_monitor_links", c, 0.2, None)
        try:
            self.json("iad_cell_link_monitor_json", "iad_cell_link_monitor_json", c)
        finally:
            self.check("iad_cell_stop_link_monitor", c)

    def gateway(self) -> None:
        units = [native.NativeSim.open("slider", slave=s) for s in (1, 2)]
        for u in units:
            self.cleanup.append(u.close)
        arr = (ctypes.c_void_p * 2)(*(u._h() for u in units))
        line = self.handle("iad_sim_gateway_line_open", b"rcon", None, arr, 2, close="iad_sim_line_close")
        port = ctypes.c_void_p()
        self.check("iad_sim_line_port", line, ctypes.byref(port))
        bus = self.handle("iad_bus_open", port, None, close="iad_bus_close")
        self.check("iad_transport_release", port)
        self.check("iad_bus_set_port", bus, b"GW", 0, NAN, 0, native.PORT_LINE(), None)
        found = ctypes.c_int32()
        self.check("iad_bus_detect_line", bus, 0, NAN, NAN, ctypes.byref(found))
        ln = self.handle("iad_bus_line", bus, close="iad_line_close")
        self.json("iad_line_json(rcon)", "iad_line_json", ln)
        gw = self.handle("iad_line_gateway", ln, close="iad_gateway_close")
        self.json("iad_gateway_info_json", "iad_gateway_info_json", gw)
        self.json("iad_gateway_read_monitor_json", "iad_gateway_read_monitor_json", gw, b"0,1")
        ax = self.handle("iad_axis_open_bus", bus, 1, None, close="iad_axis_close")
        self.check("iad_axis_connect", ax, 1, ctypes.byref(native.Identity()))
        self.json("iad_axis_gateway_status_json(rcon)", "iad_axis_gateway_status_json", ax)
        self.json("iad_axis_gateway_monitor_json(rcon)", "iad_axis_gateway_monitor_json", ax, None)
        buses = (ctypes.c_void_p * 1)(bus)
        co = native.CellOptions()
        self.check("iad_cell_options_init", ctypes.byref(co))
        co.identify = 0
        cell = self.handle("iad_cell_open_buses", buses, 1, None, b"1,2", ctypes.byref(co), None, close="iad_cell_close")
        self.json("iad_cell_gateway_status_json(rcon)", "iad_cell_gateway_status_json", cell)
        self.json("iad_cell_gateway_monitor_json(rcon)", "iad_cell_gateway_monitor_json", cell, b"GW", None)


def sample() -> dict[str, dict[str, str]]:
    with tempfile.TemporaryDirectory(prefix="iad-json-keys-") as tmp:
        return _Scenario(tmp).run()


# ====================================================================== the test


def _ready() -> str | None:
    if not KEYS_FILE.is_file():
        return "core/abi/json_keys.json is in a source checkout only"
    try:
        native.load()
    except native.NativeLibraryError as exc:
        return str(exc).splitlines()[0]
    return None


SKIP = _ready()


@unittest.skipIf(SKIP is not None, SKIP or "")
class JsonKeysTest(unittest.TestCase):
    """Every field of core/abi/json_keys.json is still written, with its type."""

    def test_no_json_field_is_removed_renamed_or_retyped(self) -> None:
        committed = json.loads(KEYS_FILE.read_text(encoding="utf-8"))["outputs"]
        now = sample()
        self.assertEqual(differences(committed, now), [])

    def test_the_comparison_catches_a_removed_and_a_retyped_field(self) -> None:
        committed = {"f": {"$": "object", "$.a": "int", "$.b": "string", "$.c": "float|null", "$.rows": "array",
                           "$.rows[]": "object", "$.rows[].x": "int"}}
        same = {"f": {"$": "object", "$.a": "int", "$.b": "string", "$.c": "null", "$.rows": "array",
                      "$.rows[]": "object", "$.rows[].x": "int", "$.new": "bool"}}
        self.assertEqual(differences(committed, same), [])  # null is no retype, an added field passes
        gone = {"f": {k: v for k, v in same["f"].items() if k != "$.b"}}
        self.assertEqual(len(differences(committed, gone)), 1)
        retyped = {"f": {**same["f"], "$.a": "float"}}
        self.assertEqual(len(differences(committed, retyped)), 1)
        empty = {"f": {"$": "object", "$.a": "int", "$.b": "string", "$.c": "null", "$.rows": "array"}}
        self.assertEqual(differences(committed, empty), [])  # no element now: the element fields cannot be seen

    def test_shape(self) -> None:
        self.assertEqual(_flat(shape({"a": 1, "b": [1.0, None], "m": {"SIM:1": {"x": "s"}, "SIM:2": {"x": "t"}}})),
                         {"$": "object", "$.a": "int", "$.b": "array", "$.b[]": "float|null", "$.m": "object",
                          "$.m{}": "object", "$.m{}.x": "string"})  # fmt: skip


def update() -> int:
    outputs = sample()
    committed = json.loads(KEYS_FILE.read_text(encoding="utf-8")) if KEYS_FILE.is_file() else {}
    old = committed.get("outputs", {})
    for problem in differences(old, outputs):
        print(f"note: {problem}")
    document = {
        "about": "The fields of every JSON output of the stable C ABI (core/include/iai_actuator_driver.h), sampled on "
                 "the native simulator by bindings/python/tests/test_abi_json_keys.py; within an ABI major a field "
                 "is only added, never removed, renamed or retyped. Regenerate with: python "
                 "tests/test_abi_json_keys.py --update (from bindings/python). \"requests\": the keys the parser "
                 "of each JSON request of the header reads (a string: the output the document is), kept by hand and "
                 "checked against the parsers by tests/test_abi_request_keys.py.",
        "abi": "{}.{}".format(*native.abi_version()),
        "outputs": outputs,
        "requests": committed.get("requests", {}),  # kept by hand: --update leaves them as they are
    }
    KEYS_FILE.parent.mkdir(parents=True, exist_ok=True)
    KEYS_FILE.write_bytes((json.dumps(document, indent=1, sort_keys=False) + "\n").encode("utf-8"))
    print(f"wrote {KEYS_FILE} ({len(outputs)} outputs, {sum(len(v) for v in outputs.values())} fields)")
    return 0


if __name__ == "__main__":
    if "--update" in sys.argv[1:]:
        sys.exit(update())
    unittest.main()
