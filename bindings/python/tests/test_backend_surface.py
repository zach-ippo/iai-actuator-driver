"""The public API keeps the 1.0 surface, and the backend in use is visible.

API snapshot: the public surface of the 1.0 driver is kept in ``tests/api_snapshot.json`` (written from release/1.0.x,
the last release with the pure-Python edition, on its native backend), so the check that compared the two editions
outlived the pure one. For each public module (:data:`MODULES`, ``iai_actuator_driver`` itself included) it holds the
``__all__`` and each public name: its kind (class, function, logger, value, object) and the module that defines it; a
function's signature (parameter names, kinds, default reprs and annotations); a class's public bases, constructor
signature, dataclass fields and public members (a method with its signature, an enum member with its value); a
constant's value (a long one as a digest). A name exported under two names (``iai_actuator_driver.Cell`` and
``iai_actuator_driver.cell.Cell``) is the very same object. These tests compare the package with it: a name that is gone
or changed fails, an added one passes (so do a new parameter with a default after the old ones, a new dataclass field
after the old ones, a new member, a new public base).

A deliberate change of the API writes the snapshot again (from ``bindings/python``)::

    python tests/test_backend_surface.py --write [PACKAGE_FOLDER]

``PACKAGE_FOLDER`` is the folder that holds the ``iai_actuator_driver`` to read (default: this checkout's
``bindings/python``); the package loads its native library as usual (``IAI_ACTUATOR_DRIVER_NATIVE``; a 1.0.x tree needs
``IAI_DRIVER_BACKEND=native`` and the DLL built from that tree).

Visible backend: ``iai-actuator-driver --version`` names it with the core and ABI versions, and the ``iai-check`` report
records it.

No hardware and no serial port.
"""

from __future__ import annotations

import contextlib
import dataclasses
import datetime as dt
import enum
import functools
import hashlib
import importlib
import inspect
import io
import json
import logging
import math
import re
import sys
import tempfile
import types
import unittest
from collections.abc import Iterable, Mapping
from pathlib import Path, PurePath
from typing import Any
from unittest import mock

if __name__ == "__main__":  # run as a script (--write): the package of this checkout, or the one in the folder given
    _FOLDER = sys.argv[2] if sys.argv[1:2] == ["--write"] and len(sys.argv) > 2 else Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(Path(_FOLDER).resolve()))

import iai_actuator_driver  # noqa: E402
from iai_actuator_driver import cli  # noqa: E402

PKG = "iai_actuator_driver"
SNAPSHOT_FILE = Path(__file__).resolve().with_name("api_snapshot.json")
MODULES = ("", "alarms", "cell", "check", "con_params", "coord", "ec", "errors", "gateway", "info", "modbus", "monitor",
           "recovery", "registers", "sim", "sim_fixtures", "sim_line", "testing", "values")  # fmt: skip
"""The public modules (``""``: the package itself). ``cli`` (its API is the command line) and ``native`` (the ctypes
binding, checked against the C header by ``test_native_binding``) are not part of it."""
NO_VALUE = (f"{PKG}:__version__",)
"""Values that change on purpose with each release: their kind is kept, not their value."""
PRIVATE_FOLDERS = ("_pure", "_native", "_shared")
"""Implementation folders: ``iai_actuator_driver._shared.coord`` defines names of ``iai_actuator_driver.coord``."""
DATA = (int, float, complex, str, bytes, bool, tuple, list, dict, set, frozenset, type(None), PurePath, enum.Enum)
PROTOCOL_DUNDERS = ("__enter__", "__exit__", "__iter__", "__next__", "__len__", "__getitem__", "__contains__",
                    "__call__", "__bool__")  # fmt: skip
LONG_VALUE = 200
"""A value whose canonical JSON is longer than this is kept as its digest."""
KINDS = {inspect.Parameter.POSITIONAL_ONLY: "pos", inspect.Parameter.POSITIONAL_OR_KEYWORD: "any",
         inspect.Parameter.VAR_POSITIONAL: "*args", inspect.Parameter.KEYWORD_ONLY: "kw",
         inspect.Parameter.VAR_KEYWORD: "**kw"}  # fmt: skip


# =========================================================================== the surface of a module


def owner(value: Any) -> str:
    """The public module that defines ``value`` (``iai_actuator_driver.coord`` for ``..._shared.coord``)."""
    module = getattr(value, "__module__", None) or ""
    return ".".join(part for part in module.split(".") if part not in PRIVATE_FOLDERS)


def is_api(name: str, value: Any) -> bool:
    """A public name of the driver's API: a constant, a logger, or a class, function or object of the driver (not a
    module, nor something imported from the standard library such as ``Path`` or ``Any``)."""
    if name.startswith("_") or name == "TYPE_CHECKING" or isinstance(value, types.ModuleType):
        return False
    if isinstance(value, (logging.Logger, *DATA)):
        return True
    module = getattr(value, "__module__", None) or ""
    return module == PKG or module.startswith(PKG + ".")


def surface(module: types.ModuleType) -> dict[str, Any]:
    """``{name: value}`` of the public API of a module (its ``__all__`` and every public driver name)."""
    names = {n for n in dir(module) if is_api(n, getattr(module, n))} | set(getattr(module, "__all__", ()))
    return {n: getattr(module, n) for n in sorted(names) if hasattr(module, n)}


def _key(item: Any) -> str:
    return json.dumps(item, sort_keys=True)


def canon(value: Any) -> Any:
    """``value`` as deterministic JSON: sets and dict items sorted, the Python types kept apart (250 and 250.0, a tuple
    and a list, an enum member and its value)."""
    if isinstance(value, enum.Enum):
        return {"enum": f"{type(value).__qualname__}.{value.name}"}
    if value is None or isinstance(value, (bool, str, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else {"float": repr(value)}
    if isinstance(value, complex):
        return {"complex": repr(value)}
    if isinstance(value, bytes):
        return {"bytes": value.hex()}
    if isinstance(value, PurePath):  # an absolute path depends on the machine (a checkout's folder)
        return {"path": "<absolute>" if value.is_absolute() else value.as_posix()}
    if isinstance(value, (list, tuple)):
        return {type(value).__name__: [canon(v) for v in value]}
    if isinstance(value, (set, frozenset)):
        return {type(value).__name__: sorted((canon(v) for v in value), key=_key)}
    if isinstance(value, dict):
        return {type(value).__name__: sorted(([canon(k), canon(v)] for k, v in value.items()), key=_key)}
    if isinstance(value, (dt.date, dt.time)):
        return {type(value).__name__: value.isoformat()}
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return {type(value).__qualname__: {f.name: canon(getattr(value, f.name)) for f in dataclasses.fields(value)}}
    if callable(value) and hasattr(value, "__qualname__"):
        return {"callable": f"{owner(value)}.{value.__qualname__}"}
    return {"object": type(value).__qualname__}


def type_name(value: Any) -> str:
    """The type of ``value`` (``Path`` for a ``WindowsPath`` or a ``PosixPath``)."""
    if isinstance(value, PurePath):
        return "Path" if isinstance(value, Path) else "PurePath"
    return type(value).__qualname__


def value_entry(value: Any) -> dict[str, Any]:
    text = json.dumps(canon(value), sort_keys=True, ensure_ascii=True, separators=(",", ":"))
    if len(text) <= LONG_VALUE:
        return {"type": type_name(value), "value": json.loads(text)}
    return {"type": type_name(value), "digest": hashlib.sha256(text.encode()).hexdigest()[:20], "n": len(text)}


def default_text(value: Any) -> str | None:
    """A parameter's default as text (``None``: no default); a sentinel shows as its type."""
    if value is inspect.Parameter.empty:
        return None
    if isinstance(value, DATA):
        return json.dumps(canon(value), sort_keys=True)
    if callable(value) and hasattr(value, "__qualname__"):
        return f"<{owner(value)}.{value.__qualname__}>"
    return f"<{type(value).__qualname__}>"


def annotation_text(value: Any) -> str | None:
    if value is inspect.Parameter.empty or value is inspect.Signature.empty:
        return None
    return re.sub(r"\s+", "", value if isinstance(value, str) else inspect.formatannotation(value))


def signature(fn: Any) -> dict[str, Any] | None:
    """``{"params": [[name, kind, annotation, default], ...], "returns": annotation}``, or ``None`` when ``fn`` has no
    signature to read (a builtin)."""
    try:
        sig = inspect.signature(fn)
    except (TypeError, ValueError):
        return None
    params = [[p.name, KINDS[p.kind], annotation_text(p.annotation), default_text(p.default)]
              for p in sig.parameters.values()]  # fmt: skip
    return {"params": params, "returns": annotation_text(sig.return_annotation)}


def driver_classes(cls: type) -> list[type]:
    """The driver classes of ``cls.__mro__``, base first."""
    return [k for k in reversed(cls.__mro__) if (getattr(k, "__module__", "") or "").startswith(PKG)]


def class_members(cls: type) -> dict[str, Any]:
    """Public members (and the protocol dunders) a driver class and its driver bases define."""
    members: dict[str, Any] = {}
    for klass in driver_classes(cls):
        for name in vars(klass):
            if not name.startswith("_") or name in PROTOCOL_DUNDERS:
                members[name] = getattr(cls, name)
    return members


def public_bases(cls: type) -> list[str]:
    """The public driver and builtin classes of ``cls.__mro__`` (``isinstance`` and ``except`` see these; a private
    helper base or a typing base is an implementation detail)."""
    return [k.__qualname__ for k in cls.__mro__[1:] if k is not object and not k.__qualname__.startswith("_")
            and ((k.__module__ or "").startswith(PKG) or k.__module__ == "builtins")]  # fmt: skip


def own_constructor(cls: type) -> bool:
    """Whether the driver defines how ``cls`` is built (its ``__init__`` / ``__new__``, or a dataclass's): a constructor
    inherited from Python (an exception's, an enum's) has a signature that changes with the Python version."""
    if issubclass(cls, enum.Enum):
        return False
    if dataclasses.is_dataclass(cls):
        return True
    for klass in driver_classes(cls):
        for name in ("__init__", "__new__"):
            fn = vars(klass).get(name)
            fn = getattr(fn, "__func__", fn)
            if fn is not None and (getattr(fn, "__module__", "") or "").startswith(PKG):
                return True
    return False


def member_entry(cls: type, value: Any) -> dict[str, Any]:
    if isinstance(cls, enum.EnumMeta) and isinstance(value, cls):
        return {"kind": "member", **value_entry(value.value)}
    if isinstance(value, type):
        return {"kind": "class"}
    if callable(value):
        return {"kind": "method", "sig": signature(value)}
    return {"kind": "attribute"}


def name_entry(value: Any) -> dict[str, Any]:
    """The entry of one public name (its class members are separate entries)."""
    if isinstance(value, logging.Logger):
        return {"kind": "logger", "name": value.name}
    if isinstance(value, type):
        entry: dict[str, Any] = {"kind": "class", "owner": owner(value), "qualname": value.__qualname__,
                                 "bases": public_bases(value)}  # fmt: skip
        if own_constructor(value):
            entry["init"] = signature(value)
        if dataclasses.is_dataclass(value):
            entry["fields"] = [f.name for f in dataclasses.fields(value)]
        return entry
    if isinstance(value, (types.FunctionType, types.BuiltinFunctionType, types.MethodType, functools.partial)):
        return {"kind": "function", "owner": owner(value), "qualname": getattr(value, "__qualname__", ""),
                "sig": signature(value)}  # fmt: skip
    if isinstance(value, DATA):
        return {"kind": "value", **value_entry(value)}
    origin = getattr(value, "__origin__", None)
    if origin is not None:  # a type alias such as Callable[[str, bytes], None] (its own type changes with Python)
        return {"kind": "alias", "origin": f"{getattr(origin, '__module__', '')}.{getattr(origin, '__qualname__', '')}"}
    if isinstance(value, (dt.date, dt.time)) or dataclasses.is_dataclass(value):
        return {"kind": "object", **value_entry(value)}
    return {"kind": "object", "type": type(value).__qualname__}


def public_modules(names: Iterable[str] = MODULES) -> dict[str, types.ModuleType]:
    return {f"{PKG}.{m}" if m else PKG: importlib.import_module(f"{PKG}.{m}" if m else PKG) for m in names}


def entries(modules: Mapping[str, types.ModuleType], *, share: bool = False) -> tuple[dict[str, Any], dict[str, Any]]:
    """``({key: entry}, {key: object})`` of the public surface of ``modules`` (``{public name: module}``). A key is
    ``module:__all__``, ``module:Name`` or ``module:Class.member``. ``share``: a class or function exported under
    several names gets its details once, under the name in its own module (else the first), and ``{"same_as": key}``
    under the others."""
    names: dict[str, Any] = {}
    for public, module in modules.items():
        for name, value in surface(module).items():
            names[f"{public}:{name}"] = value
    home: dict[int, str] = {}  # id of a shared class or function -> the key that gets its details
    if share:
        shared = {k: v for k, v in names.items() if isinstance(v, (type, types.FunctionType))}
        for key, value in shared.items():  # its own module's name first
            if key == f"{owner(value)}:{value.__qualname__}":
                home.setdefault(id(value), key)
        for key, value in shared.items():
            home.setdefault(id(value), key)
    out: dict[str, Any] = {}
    objects: dict[str, Any] = {}
    for public, module in modules.items():
        exported = getattr(module, "__all__", None)
        if exported is not None:
            out[f"{public}:__all__"] = {"kind": "__all__", "names": sorted(exported)}
    for key, value in names.items():
        objects[key] = value
        if share and id(value) in home and home[id(value)] != key:
            out[key] = {"same_as": home[id(value)]}
            continue
        entry = name_entry(value)
        if key in NO_VALUE:
            entry.pop("value", None)
            entry.pop("digest", None)
            entry.pop("n", None)
        out[key] = entry
        if isinstance(value, type):
            for member, member_value in class_members(value).items():
                out[f"{key}.{member}"] = member_entry(value, member_value)
    return out, objects


# =========================================================================== the comparison


def _params(sig: Mapping[str, Any]) -> str:
    def one(p: list[Any]) -> str:
        name, kind, annotation, default = p
        text = {"*args": "*", "**kw": "**"}.get(kind, "") + name + (f": {annotation}" if annotation else "")
        return text + (f" = {default}" if default is not None else "") + (" (keyword-only)" if kind == "kw" else "")

    return "(" + ", ".join(one(p) for p in sig["params"]) + ")" + (f" -> {sig['returns']}" if sig["returns"] else "")


def signature_problems(label: str, want: Mapping[str, Any] | None, have: Mapping[str, Any] | None) -> list[str]:
    """Why ``have`` cannot stand in for ``want``: a parameter gone, renamed, reordered, of another kind or default, or
    with another annotation (when both have one), another return annotation, or a new parameter without a default or
    before an old positional one. ``[]``: compatible."""
    if want is None:
        return []
    if have is None:
        return [f"{label}: signature {_params(want)} in the snapshot, none now"]
    old = {p[0] for p in want["params"]}
    kept = [p for p in have["params"] if p[0] in old]
    same = len(kept) == len(want["params"]) and all(
        a[:2] == b[:2] and a[3] == b[3] and (a[2] is None or b[2] is None or a[2] == b[2])
        for a, b in zip(want["params"], kept))  # fmt: skip
    if not same:
        return [f"{label}: signature {_params(want)} in the snapshot, {_params(have)} now"]
    problems = []
    last_positional = max((i for i, p in enumerate(have["params"]) if p[0] in old and p[1] in ("pos", "any")),
                          default=-1)  # fmt: skip
    for i, (name, kind, _, default) in enumerate(have["params"]):
        if name in old or kind in ("*args", "**kw"):
            continue
        if default is None:
            problems.append(f"{label}: new parameter {name} has no default ({_params(have)})")
        elif kind in ("pos", "any") and i < last_positional:
            problems.append(f"{label}: new parameter {name} comes before an old positional one ({_params(have)})")
    ra, rb = want["returns"], have["returns"]
    if ra and rb and ra != rb:
        problems.append(f"{label}: returns {ra} in the snapshot, {rb} now")
    return problems


def entry_problems(key: str, want: Mapping[str, Any], have: Mapping[str, Any]) -> list[str]:
    """Why the entry ``have`` (now) cannot stand in for ``want`` (the snapshot's) under ``key``."""
    if want.get("kind") != have.get("kind"):
        return [f"{key}: a {want.get('kind')} in the snapshot, a {have.get('kind')} now"]
    kind = want["kind"]
    if kind == "__all__":
        gone = sorted(set(want["names"]) - set(have["names"]))
        return [f"{key}: {gone} no longer exported"] if gone else []
    if kind in ("function", "method"):
        problems = []
        if kind == "function" and (want["owner"], want["qualname"]) != (have["owner"], have["qualname"]):
            problems.append(f"{key}: {want['owner']}.{want['qualname']} in the snapshot, "
                            f"{have['owner']}.{have['qualname']} now")  # fmt: skip
        return problems + signature_problems(key, want["sig"], have["sig"])
    if kind == "class":
        problems = []
        if (want["owner"], want["qualname"]) != (have["owner"], have["qualname"]):
            problems.append(f"{key}: class {want['owner']}.{want['qualname']} in the snapshot, "
                            f"{have['owner']}.{have['qualname']} now")  # fmt: skip
        gone = [b for b in want["bases"] if b not in have["bases"]]
        if gone:
            problems.append(f"{key}: no longer a subclass of {gone} (bases now {have['bases']})")
        if "init" in want:
            problems += signature_problems(key, want["init"], have.get("init"))
        fields = want.get("fields")
        if fields is not None and have.get("fields", [])[: len(fields)] != fields:
            problems.append(f"{key}: dataclass fields {fields} in the snapshot, {have.get('fields')} now")
        return problems
    if want != have:
        return [f"{key}: {json.dumps(want, sort_keys=True)[:160]} in the snapshot, "
                f"{json.dumps(have, sort_keys=True)[:160]} now"]  # fmt: skip
    return []


def surface_problems(want: Mapping[str, Any], have: Mapping[str, Any], objects: Mapping[str, Any]) -> list[str]:
    """Every way the surface now (``have``, unshared, with its ``objects``) falls short of the snapshot (``want``)."""
    problems = []
    for key, entry in sorted(want.items()):
        if key not in have:
            problems.append(f"{key}: gone")
        elif "same_as" in entry:
            other = entry["same_as"]
            if other in objects and objects[key] is not objects[other]:
                problems.append(f"{key}: no longer the same object as {other}")
        else:
            problems += entry_problems(key, entry, have[key])
    return problems


def write_snapshot(path: Path = SNAPSHOT_FILE) -> int:
    """The snapshot of the package imported now, as JSON with one line per key (a change shows as the lines it touches).
    Returns the number of keys."""
    data, _ = entries(public_modules(), share=True)
    about = (f"the public API of iai_actuator_driver {iai_actuator_driver.__version__} (backend "
             f"{iai_actuator_driver.BACKEND}), written by tests/test_backend_surface.py --write")  # fmt: skip
    lines = [f"  {json.dumps(k)}: {json.dumps(v, sort_keys=True, ensure_ascii=True, separators=(',', ':'))}"
             for k, v in sorted(data.items())]  # fmt: skip
    text = "{\n" + f' "about": {json.dumps(about)},\n "names": {{\n' + ",\n".join(lines) + "\n }\n}\n"
    path.write_text(text, encoding="utf-8", newline="\n")
    return len(data)


def read_snapshot() -> dict[str, Any]:
    return json.loads(SNAPSHOT_FILE.read_text(encoding="utf-8"))


# =========================================================================== the tests


class ApiSnapshotTest(unittest.TestCase):
    """The package against ``tests/api_snapshot.json``."""

    def test_the_snapshot_covers_every_public_module(self) -> None:
        snapshot = read_snapshot()
        self.assertIn("1.0", snapshot["about"])
        covered = {key.partition(":")[0] for key in snapshot["names"]}
        self.assertEqual(covered, set(public_modules()))

    def test_no_public_name_is_gone_or_changed(self) -> None:
        have, objects = entries(public_modules())
        problems = surface_problems(read_snapshot()["names"], have, objects)
        self.assertEqual(problems, [], "\n".join(problems))


def fake_module(source: str, name: str = "fake") -> dict[str, types.ModuleType]:
    module = types.ModuleType(f"{PKG}.{name}")
    with mock.patch.dict(sys.modules, {module.__name__: module}):  # a dataclass looks its module up
        exec(compile(source, f"<{name}>", "exec"), module.__dict__)
    return {module.__name__: module}


class SurfaceComparisonTest(unittest.TestCase):
    """The comparison itself, on two small modules: it finds each kind of change and passes additions."""

    OLD = '''
import dataclasses
import datetime as dt
import enum
import logging
from pathlib import Path
__all__ = ["Bus", "open_bus", "LIMIT", "Mode", "Point"]
log = logging.getLogger("iai_actuator_driver.fake")
LIMIT = 125
NAMES = frozenset({"a", "b"})
class Mode(enum.IntEnum):
    SLOW = 1
    FAST = 2
@dataclasses.dataclass
class Point:
    x: float
    y: float = 0.0
class Bus:
    def __init__(self, port: str, *, retries: int = 2) -> None: ...
    def read(self, address: int, count: int = 1) -> list[int]: ...
    @property
    def closed(self) -> bool: ...
    def __enter__(self) -> "Bus": ...
def open_bus(port: str, timeout: float = 0.5) -> Bus: ...
Port = Bus
'''

    def problems(self, new: str) -> list[str]:
        want, _ = entries(fake_module(self.OLD), share=True)
        have, objects = entries(fake_module(new))
        return surface_problems(json.loads(json.dumps(want)), have, objects)

    def test_the_same_module_has_no_problems(self) -> None:
        self.assertEqual(self.problems(self.OLD), [])

    def test_additions_pass(self) -> None:
        new = (self.OLD.replace('"Point"]', '"Point", "extra"]')
               .replace("count: int = 1)", "count: int = 1, slave: int = 1)")
               .replace("retries: int = 2)", "retries: int = 2, gap: float = 0.0)")
               .replace("    y: float = 0.0\n", "    y: float = 0.0\n    z: float = 0.0\n")
               .replace("    FAST = 2\n", "    FAST = 2\n    FASTER = 3\n")
               .replace("def open_bus(", "def extra() -> None: ...\ndef open_bus("))  # fmt: skip
        self.assertEqual(self.problems(new), [])

    def test_each_change_is_found(self) -> None:
        new = (self.OLD.replace('"Mode", "Point"]', '"Point"]')
               .replace("retries: int = 2", "retries: int = 3")
               .replace("count: int = 1", "size: int = 1")
               .replace("LIMIT = 125", "LIMIT = 124.0")
               .replace("    FAST = 2\n", "    FAST = 3\n")
               .replace('    def __enter__(self) -> "Bus": ...\n', "")
               .replace("    x: float\n", "    x: float\n    w: float\n")
               .replace("timeout: float = 0.5)", "timeout: float = 0.5, *, slave: int)")
               .replace("Port = Bus", "class Port(Bus): ...")
               .replace('frozenset({"a", "b"})', 'frozenset({"a"})'))  # fmt: skip
        text = "\n".join(self.problems(new))
        for expected in ("iai_actuator_driver.fake:__all__: ['Mode'] no longer exported",
                         "iai_actuator_driver.fake:Bus: signature (port: str, retries: int = 2 (keyword-only))",
                         "iai_actuator_driver.fake:Bus.read: signature", "iai_actuator_driver.fake:LIMIT: ",
                         "iai_actuator_driver.fake:Mode.FAST: ", "iai_actuator_driver.fake:Bus.__enter__: gone",
                         "iai_actuator_driver.fake:Point: dataclass fields", "iai_actuator_driver.fake:NAMES: ",
                         "iai_actuator_driver.fake:open_bus: new parameter slave has no default",
                         "iai_actuator_driver.fake:Port: no longer the same object as iai_actuator_driver.fake:Bus"):
            with self.subTest(expected=expected):
                self.assertIn(expected, text)

    def test_a_shared_name_is_kept_once(self) -> None:
        want, _ = entries(fake_module(self.OLD), share=True)
        self.assertEqual(want["iai_actuator_driver.fake:Port"], {"same_as": "iai_actuator_driver.fake:Bus"})
        self.assertNotIn("iai_actuator_driver.fake:Port.read", want)
        self.assertEqual(want["iai_actuator_driver.fake:log"], {"kind": "logger", "name": "iai_actuator_driver.fake"})


class PackageExportsTest(unittest.TestCase):
    def test_every_exported_name_resolves(self) -> None:
        self.assertEqual([n for n in iai_actuator_driver.__all__ if not hasattr(iai_actuator_driver, n)], [])
        self.assertEqual(iai_actuator_driver.BACKEND, iai_actuator_driver.backend_info()["backend"])


class BackendVisibleTest(unittest.TestCase):
    def test_version_names_the_backend(self) -> None:
        out = io.StringIO()
        with contextlib.redirect_stdout(out), self.assertRaises(SystemExit) as stop:
            cli.build_parser().parse_args(["--version"])
        self.assertEqual(stop.exception.code, 0)
        text = out.getvalue().strip()
        self.assertTrue(text.startswith(f"iai-actuator-driver {iai_actuator_driver.__version__} "
                                        f"(backend {iai_actuator_driver.BACKEND}"), text)  # fmt: skip
        self.assertIn(", ABI ", text)

    def test_version_on_native_adds_the_core_and_abi_versions(self) -> None:
        info = {"backend": "native", "abi": (0, 3), "core_version": "0.12.0b1", "library": "C:\\100%\\x.dll"}
        with mock.patch.object(cli, "backend_info", return_value=info):
            text = cli.version_text()
        self.assertEqual(text, f"%(prog)s {iai_actuator_driver.__version__} (backend native, core 0.12.0b1, ABI 0.3)")

    def test_check_report_records_the_backend(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "report.json"
            out = io.StringIO()
            with contextlib.redirect_stderr(io.StringIO()):
                code = cli.main(["--sim", "check", "--report", str(path)], out=out)
            self.assertEqual(code, 0, out.getvalue())
            report = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(report["backend"], iai_actuator_driver.BACKEND)
        self.assertEqual(report["driver_version"], iai_actuator_driver.__version__)


if __name__ == "__main__":
    if sys.argv[1:2] == ["--write"]:
        print(f"{write_snapshot()} names of {iai_actuator_driver.__file__} -> {SNAPSHOT_FILE}")
    else:
        unittest.main()
