#!/usr/bin/env python3
"""The C ABI manifest: every declaration of the two C headers in one committed text file, and its checks.

    python core/abi/manifest.py check     the headers against manifest.txt, native.py's signature tables and the C#
                                          DllImports (exit 1 and a list of differences when anything disagrees)
    python core/abi/manifest.py update    rewrite manifest.txt from the headers (after a deliberate change of the ABI)

(``python core/build.py manifest [--update]`` runs the same.) The manifest lists, per header and in header order, the
constants, the type names, the struct fields, the callback types and the function signatures (parameter types without
their names). Its diff is the review of an ABI change. ``update`` refuses to remove or change a line of the stable
header within one ABI major from 1 on (only additions pass) unless ``--allow-break`` is given. manifest-1.0.txt is the
frozen baseline (the 1.0.0 manifest, never edited again) that closes what ``update`` leaves open: the gate test
(bindings/python/tests/test_abi_manifest_gate.py) runs ``breaking_changes(baseline, manifest.txt)``, which must be empty
whatever ``--allow-break`` did, and lets the stable section differ from the baseline only once IAD_ABI_VERSION_MINOR is
above 0 (an addition to the stable header is a new minor).

The binding checks compare what crosses the boundary, parameter by parameter, by kind: i32 (int32_t, iad_result,
iad_bool, iad_tribool), u32, i64, u64, u16, u8, f64, size (size_t), ptr (any data pointer) and fn (a callback). Python:
native._SIGNATURES binds the stable header and native._TESTING_SIGNATURES the testing header, each in header order.
C#: every function of both headers has a DllImport (by entry point; a function may have more than one), the stable
ones in class NativeMethods, the testing ones in class NativeTestingMethods, every one CallingConvention.Winapi (IAD_CALL:
stdcall on 32-bit Windows, the one convention elsewhere), and none names a function
the headers lack. These replace the hand-kept function counts of the binding tests.
"""

from __future__ import annotations

import argparse
import difflib
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

CORE = Path(__file__).resolve().parents[1]
REPO = CORE.parent
STABLE = "iai_actuator_driver.h"
TESTING = "iai_actuator_driver_testing.h"
HEADERS = {STABLE: CORE / "include" / STABLE, TESTING: CORE / "include" / TESTING}
MANIFEST = CORE / "abi" / "manifest.txt"
BASELINE = CORE / "abi" / "manifest-1.0.txt"  # frozen: the stable 1.0 baseline of the gate test
PYTHON_ROOT = REPO / "bindings" / "python"
CSHARP_ROOT = REPO / "bindings" / "dotnet" / "src" / "IaiActuatorDriver"
CSHARP_CLASSES = {STABLE: "NativeMethods", TESTING: "NativeTestingMethods"}

# ====================================================================== the headers


@dataclass
class Header:
    name: str
    constants: list[tuple[str, str]] = field(default_factory=list)  # (name, value text)
    types: list[tuple[str, str]] = field(default_factory=list)  # (name, "int32_t" / "opaque")
    structs: list[tuple[str, list[str]]] = field(default_factory=list)  # (name, ["double stroke_min", ...])
    callbacks: list[tuple[str, str, list[str]]] = field(default_factory=list)  # (name, return type, param types)
    functions: list[tuple[str, str, list[str]]] = field(default_factory=list)  # (name, return type, param types)


def _strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    text = re.sub(r"//[^\n]*", "", text)
    return text.replace("\\\n", " ")  # macro continuation lines


def _norm_type(text: str) -> str:
    """'const  iad_axis_options *' -> 'const iad_axis_options*'; 'iad_sim * const *' -> 'iad_sim* const*'."""
    text = " ".join(text.split())
    text = re.sub(r"\s*\*", "*", text)
    return re.sub(r"\*(?=[A-Za-z_])", "* ", text)


def _param_types(params: str) -> list[str]:
    params = " ".join(params.split())
    if params in ("", "void"):
        return []
    out = []
    for p in params.split(","):
        m = re.fullmatch(r"(.*?[\s*])([A-Za-z_]\w*)", p.strip())
        if m is None or not m.group(1).strip():
            raise ValueError(f"cannot read the parameter {p.strip()!r}")
        out.append(_norm_type(m.group(1)))
    return out


def parse_header(name: str, path: Path | None = None) -> Header:
    text = _strip_comments((path or HEADERS[name]).read_text(encoding="utf-8"))
    h = Header(name)
    enum = re.search(r"enum iad_result_code \{(.*?)\};", text, flags=re.S)
    if enum:
        for c, v in re.findall(r"(IAD_\w+)\s*=\s*(-?\d+)", enum.group(1)):
            h.constants.append((c, v))
    for m in re.finditer(r"^[ \t]*#[ \t]*define[ \t]+(IAD_\w+(?:\([^)]*\))?)[ \t]*(.*)$", text, flags=re.M):
        if m.group(1) in ("IAD_API", "IAD_CALL"):
            continue
        h.constants.append((m.group(1), " ".join(m.group(2).split())))
    # IAD_CALL (0.8): the calling convention is part of the ABI, so the rule that picks it is in the manifest
    call = re.search(r"#[ \t]*if[ \t]+(.+?)\n[ \t]*#[ \t]*define[ \t]+IAD_CALL[ \t]+(\w+)[ \t]*\n"
                     r"[ \t]*#[ \t]*else[ \t]*\n[ \t]*#[ \t]*define[ \t]+IAD_CALL[ \t]*\n", text)
    if call:
        h.constants.append(("IAD_CALL", f"{call.group(2)} if {' '.join(call.group(1).split())} else (empty)"))
    for m in re.finditer(r"typedef\s+(\w+)\s+(iad_\w+)\s*;", text):
        h.types.append((m.group(2), m.group(1)))
    for m in re.finditer(r"typedef\s+struct\s+(iad_\w+)\s+(iad_\w+)\s*;", text):
        h.types.append((m.group(2), "opaque struct"))
    for m in re.finditer(r"typedef\s+struct\s+(iad_\w+)\s*\{(.*?)\}\s*\1\s*;", text, flags=re.S):
        fields = []
        for f in re.findall(r"([^;{}]+);", m.group(2)):
            fm = re.fullmatch(r"(.*?[\s*])([A-Za-z_]\w*)\s*(\[[^\]]*\])?", " ".join(f.split()))
            if fm is None:
                raise ValueError(f"{m.group(1)}: cannot read the field {f.strip()!r}")
            fields.append(f"{_norm_type(fm.group(1))} {fm.group(2)}{(fm.group(3) or '').replace(' ', '')}")
        h.structs.append((m.group(1), fields))
    for m in re.finditer(r"typedef\s+([\w\s\*]+?)\s*\(\s*IAD_CALL\s*\*\s*(iad_\w+)\s*\)\s*\((.*?)\)\s*;", text,
                         flags=re.S):
        h.callbacks.append((m.group(2), _norm_type(m.group(1)), _param_types(m.group(3))))
    for m in re.finditer(r"IAD_API\s+([\w\s\*]+?)\s*IAD_CALL\s+(iad_\w+)\s*\((.*?)\)\s*;", text, flags=re.S):
        h.functions.append((m.group(2), _norm_type(m.group(1)), _param_types(m.group(3))))
    return h


def parse_headers() -> dict[str, Header]:
    return {name: parse_header(name) for name in HEADERS}


def _version(headers: dict[str, Header]) -> tuple[int, int, int]:
    consts = {n: v for h in headers.values() for n, v in h.constants}
    return int(consts["IAD_ABI_VERSION_MAJOR"]), int(consts["IAD_ABI_VERSION_MINOR"]), int(
        consts["IAD_TESTING_ABI_VERSION"])


def manifest_text(headers: dict[str, Header]) -> str:
    major, minor, testing = _version(headers)
    lines = [
        "# The C ABI manifest: generated by core/abi/manifest.py from core/include/iai_actuator_driver.h and",
        "# core/include/iai_actuator_driver_testing.h. Do not edit; after a deliberate ABI change run",
        "# 'python core/build.py manifest --update' and review the diff.",
        f"abi {major}.{minor}",
        f"testing {testing}",
    ]
    for name, h in headers.items():
        lines += ["", f"[{name}] {len(h.functions)} functions, {len(h.structs)} structs"]
        lines += [f"const {n} {v}".rstrip() for n, v in h.constants]
        lines += [f"type {n} {v}" for n, v in h.types]
        lines += [f"callback {n} {r}({', '.join(p) or 'void'})" for n, r, p in h.callbacks]
        for n, fields in h.structs:
            lines.append(f"struct {n}")
            lines += [f"    {f}" for f in fields]
        lines += [f"function {n} {r}({', '.join(p) or 'void'})" for n, r, p in h.functions]
    return "\n".join(lines) + "\n"


# ====================================================================== kinds of what crosses the boundary

_C_KINDS = {
    "int32_t": "i32", "iad_result": "i32", "iad_bool": "i32", "iad_tribool": "i32", "uint32_t": "u32",
    "int64_t": "i64", "uint64_t": "u64", "uint16_t": "u16", "uint8_t": "u8", "double": "f64", "size_t": "size",
    "void": "void",
}


def c_kind(ctype: str, callbacks: set[str]) -> str:
    if "*" in ctype:
        return "ptr"
    bare = ctype.replace("const ", "").strip()
    if bare in callbacks:
        return "fn"
    try:
        return _C_KINDS[bare]
    except KeyError:
        raise ValueError(f"no kind for the C type {ctype!r} (add it to manifest.py)") from None


def _functions(headers: dict[str, Header]) -> dict[str, list[tuple[str, str, list[str]]]]:
    """Per header: (name, return kind, parameter kinds) in header order."""
    callbacks = {n for h in headers.values() for n, _, _ in h.callbacks}
    return {name: [(n, c_kind(r, callbacks), [c_kind(p, callbacks) for p in ps]) for n, r, ps in h.functions]
            for name, h in headers.items()}


# ====================================================================== the Python binding


def _python_kinds(ctype: Any) -> set[str]:
    import ctypes

    if ctype is None:
        return {"void"}
    kinds = set()
    for t, k in ((ctypes.c_int32, "i32"), (ctypes.c_uint32, "u32"), (ctypes.c_int64, "i64"),
                 (ctypes.c_uint64, "u64"), (ctypes.c_uint16, "u16"), (ctypes.c_uint8, "u8"),
                 (ctypes.c_double, "f64"), (ctypes.c_size_t, "size")):
        if ctype is t:
            kinds.add(k)
    if isinstance(ctype, type):
        if ctype in (ctypes.c_char_p, ctypes.c_void_p) or issubclass(ctype, (ctypes._Pointer, ctypes.Array)):
            kinds.add("ptr")
        if ctype is ctypes.c_void_p or issubclass(ctype, ctypes._CFuncPtr):  # type: ignore[attr-defined]
            kinds.add("fn")
    return kinds


def python_binding() -> Any:
    """``iai_actuator_driver.native`` of this checkout, imported without the package's ``__init__`` (which loads the
    native library): its signature tables need no library, so the check runs before any build."""
    import importlib
    import types

    if "iai_actuator_driver" not in sys.modules:
        package = types.ModuleType("iai_actuator_driver")
        package.__path__ = [str(PYTHON_ROOT / "iai_actuator_driver")]  # native.py imports errors.py alone from it
        sys.modules["iai_actuator_driver"] = package
    return importlib.import_module("iai_actuator_driver.native")


def check_python(native: Any = None, headers: dict[str, Header] | None = None) -> list[str]:
    """Differences between the headers and native._SIGNATURES / native._TESTING_SIGNATURES (empty: none)."""
    if native is None:
        native = python_binding()
    funcs = _functions(headers or parse_headers())
    problems: list[str] = []
    for header, table in ((STABLE, "_SIGNATURES"), (TESTING, "_TESTING_SIGNATURES")):
        bound = list(getattr(native, table, ()))
        want = [n for n, _, _ in funcs[header]]
        have = [n for n, _, _ in bound]
        if have != want:
            for line in difflib.unified_diff(want, have, f"{header}", f"native.{table}", lineterm="", n=1):
                problems.append(f"python: {line}")
            continue
        for (name, ret, params), (_, restype, argtypes) in zip(funcs[header], bound):
            if ret not in _python_kinds(restype):
                problems.append(f"python: {name} returns {ret}, native.{table} says {restype!r}")
            if len(argtypes) != len(params):
                problems.append(f"python: {name} takes {len(params)} parameters, native.{table} {len(argtypes)}")
                continue
            for i, (kind, ctype) in enumerate(zip(params, argtypes)):
                if kind not in _python_kinds(ctype):
                    problems.append(f"python: {name} parameter {i + 1} is {kind}, native.{table} binds {ctype!r}")
    return problems


# ====================================================================== the C# binding

_CS_KINDS = {
    "int": {"i32"}, "uint": {"u32"}, "long": {"i64"}, "ulong": {"u64"}, "ushort": {"u16"}, "byte": {"u8"},
    "double": {"f64"}, "UIntPtr": {"size"}, "void": {"void"}, "IntPtr": {"ptr", "fn"},
}
_DLLIMPORT = re.compile(
    r"\[DllImport\((?:NativeMethods\.)?Library\b(?P<args>[^\]]*)\)\]\s*"
    r"(?:\[return:[^\]]*\]\s*)?"
    r"(?:internal|public|private)\s+static\s+extern\s+(?P<ret>[\w\[\]?]+)\s+(?P<name>\w+)\s*\((?P<params>[^)]*)\)\s*;",
    flags=re.S,
)


_CS_ATTRIBUTE = re.compile(r"\[\s*[A-Za-z][^\]]*\]")  # [Out], [In], [MarshalAs(..., SizeParamIndex = 2)]; not "[]"


def _cs_kinds(param: str, delegates: set[str]) -> set[str]:
    words = param.split()
    if words and words[0] in ("ref", "out", "in"):
        return {"ptr"}
    if not words:
        return set()
    t = words[0].rstrip("?")
    if t.endswith("[]"):
        return {"ptr"}
    if t in _CS_KINDS:
        return _CS_KINDS[t]
    if t.endswith("Handle"):  # SafeHandle subclasses: the token by value
        return {"ptr"}
    if t in delegates:
        return {"fn"}
    return {f"unknown C# type {t}"}


def csharp_imports(root: Path = CSHARP_ROOT) -> tuple[list[dict[str, Any]], set[str]]:
    """Every DllImport of the core library: entry point, class, file, return and parameter kinds, calling convention."""
    files = sorted(root.rglob("*.cs"))
    files = [f for f in files if "bin" not in f.parts and "obj" not in f.parts]
    texts = {f: f.read_text(encoding="utf-8-sig") for f in files}
    delegates = set()
    for text in texts.values():
        delegates.update(re.findall(r"\bdelegate\s+[\w\[\]?]+\s+(\w+)\s*\(", text))
    imports = []
    for f, text in texts.items():
        classes = [(m.start(), m.group(1)) for m in re.finditer(r"\bclass\s+(\w+)", text)]
        for m in _DLLIMPORT.finditer(text):
            entry = re.search(r'EntryPoint\s*=\s*"(\w+)"', m.group("args"))
            owner = [c for pos, c in classes if pos < m.start()]
            params = [p for p in (x.strip() for x in _CS_ATTRIBUTE.sub(" ", m.group("params")).split(",")) if p]
            imports.append({
                "entry": entry.group(1) if entry else m.group("name"),
                "method": m.group("name"),
                "class": owner[-1] if owner else "",
                "file": f.relative_to(root).as_posix(),
                "ret": _CS_KINDS.get(m.group("ret"), {f"unknown C# type {m.group('ret')}"}),
                "params": [_cs_kinds(p, delegates) for p in params],
                "winapi": "CallingConvention = Winapi" in m.group("args")
                          or "CallingConvention.Winapi" in m.group("args"),
            })
    return imports, delegates


def check_csharp(headers: dict[str, Header] | None = None, root: Path = CSHARP_ROOT) -> list[str]:
    """Differences between the headers and the C# DllImports (empty: none)."""
    funcs = _functions(headers or parse_headers())
    where = {n: (header, r, ps) for header, fs in funcs.items() for n, r, ps in fs}
    imports, _ = csharp_imports(root)
    problems: list[str] = []
    seen: set[str] = set()
    for imp in imports:
        name = imp["entry"]
        at = f"c#: {imp['file']} {imp['class']}.{imp['method']}"
        if name not in where:
            problems.append(f"{at} imports {name}, which neither header declares")
            continue
        seen.add(name)
        header, ret, params = where[name]
        if imp["class"] != CSHARP_CLASSES[header]:
            problems.append(f"{at}: a function of {header} belongs in class {CSHARP_CLASSES[header]}")
        if not imp["winapi"]:
            problems.append(f"{at}: not CallingConvention.Winapi")
        if ret not in imp["ret"]:
            problems.append(f"{at}: {name} returns {ret}, the import {sorted(imp['ret'])}")
        if len(imp["params"]) != len(params):
            problems.append(f"{at}: {name} takes {len(params)} parameters, the import {len(imp['params'])}")
            continue
        for i, (kind, kinds) in enumerate(zip(params, imp["params"])):
            if kind not in kinds:
                problems.append(f"{at}: {name} parameter {i + 1} is {kind}, the import passes {sorted(kinds)}")
    for header, fs in funcs.items():
        for n, _, _ in fs:
            if n not in seen:
                problems.append(f"c#: no DllImport of {n} ({header})")
    return problems


# ====================================================================== the committed manifest


def check_manifest(headers: dict[str, Header] | None = None, path: Path = MANIFEST) -> list[str]:
    """Differences between the headers and the committed manifest (empty: none)."""
    text = manifest_text(headers or parse_headers())
    if not path.is_file():
        return [f"manifest: {path} is missing (python core/build.py manifest --update)"]
    committed = path.read_text(encoding="utf-8")
    if committed == text:
        return []
    diff = difflib.unified_diff(committed.splitlines(), text.splitlines(), "manifest.txt (committed)",
                                "the headers", lineterm="", n=0)
    return ["manifest: the headers differ from core/abi/manifest.txt (after a deliberate ABI change: "
            "python core/build.py manifest --update)"] + [f"manifest: {line}" for line in diff]


@dataclass
class _Stable:
    """The stable header's section of a manifest text: what a major version's promise covers."""

    major: int | None = None
    minor: int | None = None
    simple: dict[tuple[str, str], str] = field(default_factory=dict)  # (kind, name) -> its line
    structs: dict[str, list[str]] = field(default_factory=dict)  # name -> its field lines, in order
    types: dict[str, str] = field(default_factory=dict)  # typedef name -> underlying type
    consts: dict[str, str] = field(default_factory=dict)  # constant -> value text


def _parse_stable(text: str) -> _Stable:
    out = _Stable()
    inside = False
    struct: list[str] | None = None
    for line in text.splitlines():
        if line.startswith("abi "):
            out.major, out.minor = (int(x) for x in line.split()[1].split("."))
        elif line.startswith("["):
            inside = line.startswith(f"[{STABLE}]")
            struct = None
        elif not inside or not line.strip():
            continue
        elif line.startswith(" "):
            if struct is not None:
                struct.append(line.strip())
        else:
            kind, _, rest = line.partition(" ")
            name, _, value = rest.partition(" ")
            if kind == "struct":
                struct = out.structs.setdefault(name, [])
                continue
            struct = None
            out.simple[(kind, name)] = line
            if kind == "const":
                out.consts[name] = value
            elif kind == "type":
                out.types[name] = value
    return out


_PRIMITIVE_SIZES = {"int32_t": 4, "uint32_t": 4, "int64_t": 8, "uint64_t": 8, "uint16_t": 2, "uint8_t": 1,
                    "double": 8, "size_t": 8, "char": 1}
_FIELD = re.compile(r"(?P<type>.+?)\s*(?P<name>[A-Za-z_]\w*)(?:\[(?P<count>[^\]]*)\])?")


def _field_size(stable: _Stable, field_line: str, callbacks: set[str]) -> int:
    """Bytes of one struct field (the layout has no implicit padding, so a struct is the sum of its fields)."""
    m = _FIELD.fullmatch(field_line)
    if m is None:
        raise ValueError(f"cannot read the field {field_line!r}")
    count = 1
    if m.group("count") is not None:
        text = m.group("count")
        value = stable.consts.get(text, text)
        try:
            count = int(value, 0)
        except ValueError:
            raise ValueError(f"cannot size the array {field_line!r}") from None
    kind = m.group("type").replace("const ", "").strip()
    seen: set[str] = set()
    while kind in stable.types and kind not in seen and stable.types[kind] in _PRIMITIVE_SIZES | stable.types:
        seen.add(kind)
        kind = stable.types[kind]
    if "*" in kind or kind in callbacks:
        size = 8
    elif kind in _PRIMITIVE_SIZES:
        size = _PRIMITIVE_SIZES[kind]
    elif kind in stable.structs:
        size = sum(_field_size(stable, f, callbacks) for f in stable.structs[kind])
    else:
        raise ValueError(f"cannot size the type of {field_line!r}")
    return size * count


def _is_reserved(field_line: str) -> bool:
    m = _FIELD.fullmatch(field_line)
    return m is not None and m.group("name").startswith("reserved") and m.group("name") != "reserved_header"


def _struct_changes(name: str, old: list[str], new: list[str], old_stable: _Stable, new_stable: _Stable) -> list[str]:
    """How a struct of the stable header may change within a major: a sized input (it starts with struct_size and
    reserved_header) keeps its fields in order and may get new ones appended at its end; any other struct (an output) is
    fixed in size, so only a reserved slot may be replaced, by fields of the same total size."""
    head = f"struct {name} / "
    if old[:2] == ["uint32_t struct_size", "uint32_t reserved_header"]:
        for i, f in enumerate(old):
            if i >= len(new):
                return [f"{head}{f.strip()} (removed)"]
            if new[i] != f:
                return [f"{head}{f.strip()} (field {i} is now {new[i].strip()}: fields are never removed, reordered "
                        f"or changed; a sized struct only gets fields appended at its end)"]
        return []
    callbacks = {k[1] for k in new_stable.simple if k[0] == "callback"}
    out: list[str] = []
    j = 0
    for f in old:
        if j < len(new) and new[j] == f:
            j += 1
            continue
        if _is_reserved(f):
            try:
                want = _field_size(old_stable, f, callbacks)
                total, k = 0, j
                while k < len(new) and total < want:
                    total += _field_size(new_stable, new[k], callbacks)
                    k += 1
            except ValueError as exc:
                return [f"{head}{f.strip()} ({exc})"]
            if k > j and total == want:
                j = k
                continue
        why = "removed, moved or changed" if j >= len(new) or new[j] != f else "changed"
        at = f" (field {j} is now {new[j].strip()})" if j < len(new) else ""
        return [f"{head}{f.strip()} ({why}{at}: an output has a fixed size, only a reserved slot may be replaced by "
                f"fields of the same total size)"]
    if j < len(new):
        out.append(f"{head}{new[j].strip()} (appended to an output: an output has a fixed size, a new field takes a "
                   f"reserved slot's place)")
    return out


def breaking_changes(old_text: str, new_text: str) -> list[str]:
    """What the stable header lost or changed within one ABI major from 1 on (only additions pass). Constants, types,
    callbacks and functions are keyed by their name and must read the same (the minor version may rise); a struct is an
    ordered field list: a sized input keeps its fields and may get new ones appended at its end, an output keeps its
    size (only a reserved slot may be given fields of the same total size). Empty when the majors differ or are 0."""
    old = _parse_stable(old_text)
    new = _parse_stable(new_text)
    if old.major is None or new.major is None or old.major != new.major or new.major < 1:
        return []
    out: list[str] = []
    for key, line in old.simple.items():
        now = new.simple.get(key)
        if now == line:
            continue
        if key == ("const", "IAD_ABI_VERSION_MINOR") and now is not None and int(now.split()[-1]) >= int(line.split()[-1]):
            continue  # a minor release raises it
        out.append(line if now is None else f"{line}  ->  {now}")
    for name, fields in old.structs.items():
        if name not in new.structs:
            out.append(f"struct {name} (removed)")
        else:
            out += _struct_changes(name, fields, new.structs[name], old, new)
    return out


def check_all() -> list[str]:
    headers = parse_headers()
    problems = check_manifest(headers)
    try:
        problems += check_python(headers=headers)
    except ImportError as exc:
        problems.append(f"python: cannot import the binding ({exc})")
    problems += check_csharp(headers)
    return problems


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="The C ABI manifest: generate it, or check the headers and both "
                                                 "bindings against it.")
    parser.add_argument("command", choices=["check", "update"])
    parser.add_argument("--allow-break", action="store_true",
                        help="update: accept removed or changed lines of the stable header within a major (>= 1)")
    args = parser.parse_args(argv)
    if args.command == "update":
        headers = parse_headers()
        text = manifest_text(headers)
        old = MANIFEST.read_text(encoding="utf-8") if MANIFEST.is_file() else ""
        broken = breaking_changes(old, text)
        if broken and not args.allow_break:
            print("refused: within an ABI major only additions pass; removed or changed in "
                  f"{STABLE}:\n  " + "\n  ".join(broken) + "\n(--allow-break when the major changes anyway)")
            return 1
        MANIFEST.parent.mkdir(parents=True, exist_ok=True)
        MANIFEST.write_bytes(text.encode("utf-8"))
        counts = ", ".join(f"{h.name}: {len(h.functions)} functions" for h in headers.values())
        print(f"wrote {MANIFEST.relative_to(REPO).as_posix()} ({counts})")
        return 0
    problems = check_all()
    for p in problems:
        print(p)
    headers = parse_headers()
    counts = ", ".join(f"{h.name} {len(h.functions)}" for h in headers.values())
    print(f"manifest check: {len(problems)} difference(s) ({counts} functions)")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
