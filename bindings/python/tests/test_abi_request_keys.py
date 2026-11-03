"""The JSON requests of the C ABI keep their key lists: what each parser reads against core/abi/json_keys.json.

core/abi/json_keys.json "requests" lists, for every JSON text the stable header (core/include/iai_actuator_driver.h)
takes (a ``const char*`` parameter named ``request`` or ``*_json``; each ``iad_call_json`` and
``iad_axis_info_call_json`` function on its own), the keys its parser reads: anywhere in the request, nested objects
included (an object keyed by data, such as a point's axis names or a port's slaves, has no keys to list). A document
the library writes itself (a backup) is named by the output it is: the keys its readers take are fields of that output
in "outputs". The test reads the parsers' source (core/src) and fails when a parser reads a key the list lacks or the
list names a key no parser reads, and when the function's own header comment (the comment of its section for the
requests a section documents) does not name a listed key as "key". The arguments of the iad_call_json functions are
documented by the list alone.

The scan follows a part of the request under a local name (`const Json& t = arg(a, "table")`, `j.find("k")`, a loop
over an array) and into any helper of core/src it is handed to, the request or a part of it (`con_param_arg(a, "p")`),
by the helper's own parameter names. A key read in another way (a key built at run time) is not seen.

The key lists are kept by hand: after a deliberate change to a parser, edit core/abi/json_keys.json "requests" and the
header comment together (test_abi_json_keys.py --update keeps them).
"""

from __future__ import annotations

import json
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CORE = ROOT.parents[1] / "core"
KEYS_FILE = CORE / "abi" / "json_keys.json"
HEADER = CORE / "include" / "iai_actuator_driver.h"
SRC = CORE / "src"

KEY = r"[a-z_][a-z0-9_]*"
# Helpers whose second argument names the text, not a key of it.
NOT_A_KEY_READ = {"need", "parse_object", "parse_backup", "int_values"}
# Where each request is parsed: (file, the text the parser starts at, the names of the request object there; None: any
# object, for a document whose keys are checked against an output). The helpers the request or a part of it is handed
# to are read too (keys_read).
_COORD = [("c_api_coord.cpp", "Request parse_request(", {"j"}),
          ("c_api_coord.cpp", "coord::Path path_of(", {"j"}),
          ("c_api_coord.cpp", "coord::Segment segment_of(", {"j", "v"})]
_BACKUP = [("recovery.cpp", "const Json* backup_identity(", None),
           ("recovery.cpp", "std::map<int, PositionEntry> backup_rows(", None),
           ("recovery.cpp", "std::map<std::string, std::string> backup_parameters(", None),
           ("recovery.cpp", "std::shared_ptr<Session> session_from_backup(const Axis& axis, const Json& backup", None),
           ("recovery.cpp", "std::optional<Json> usable_backup(", None)]
PARSERS: dict[str, list[tuple[str, str, set[str] | None]]] = {
    "iad_axis_recover": _BACKUP,
    "iad_axis_session_from_backup": _BACKUP,
    "iad_axis_session_from_info": [("c_api_recovery.cpp", "DeviceInfo identity_from_json(", {"j"})],
    "iad_axis_session_update": [("recovery.cpp", "void update_session(", {"fields"})],
    "iad_axis_restart_signs": [("c_api_recovery.cpp", "iad_result IAD_CALL iad_axis_restart_signs(", {"o"})],
    "iad_axis_set_axis_limits_json": [("c_api_axis.cpp", "iad_result IAD_CALL iad_axis_set_axis_limits_json(", {"j"})],
    "iad_axis_track": _BACKUP,
    "iad_bus_state_line_rules": [("c_api_line.cpp", "iad_result IAD_CALL iad_bus_state_line_rules(", {"rules"})],
    "iad_cell_coord_plan_json": _COORD,
    "iad_cell_coord_run_json": _COORD,
    "iad_cell_open_buses": [("c_api_cell.cpp", "slaves_by_port(const char* slaves_json", {"j"})],
    "iad_sim_gateway_line_open": [("c_api_sim.cpp", "SimGatewayOptions gateway_options_of(", {"j"})],
    "iad_sim_open": [("c_api_sim.cpp", "bool set_option_key(", {"v"})],
}
# Requests whose keys are the fields of an output (the header names that output instead of each key).
FIELDS_OF = {"iad_axis_set_axis_limits_json": "iad_axis_read_axis_limits_json"}
# Requests the comment of their header section documents (the shapes of a path, the simulator's options).
SECTION_DOCUMENTED = {"iad_cell_coord_plan_json", "iad_cell_coord_run_json", "iad_sim_open"}
CALL_FILE, INFO_FILE = "c_api_tables.cpp", "c_api_info.cpp"


# ====================================================================== reading the source


def _blank_comments(text: str) -> str:
    """The text with its comments replaced by spaces (string and character literals kept)."""
    out, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if c in "\"'":
            j = i + 1
            while j < n and text[j] != c:
                j += 2 if text[j] == "\\" else 1
            out.append(text[i:j + 1])
            i = j + 1
        elif text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append(re.sub(r"[^\n]", " ", text[i:j]))
            i = j
        else:
            out.append(c)
            i += 1
    return "".join(out)


def _block(text: str, start: int) -> str:
    """From `start` to the end of the statement or the braced block that follows it (string literals skipped)."""
    depth, i, n = 0, start, len(text)
    while i < n:
        c = text[i]
        if c in "\"'":
            i += 1
            while i < n and text[i] != c:
                i += 2 if text[i] == "\\" else 1
        elif c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return text[start:i + 1]
        elif c == ";" and depth == 0:
            return text[start:i + 1]
        i += 1
    raise AssertionError(f"no end of the block at {text[start:start + 60]!r}")


_SOURCES: dict[str, str] = {}


def source(name: str) -> str:
    if name not in _SOURCES:
        _SOURCES[name] = _blank_comments((SRC / name).read_text(encoding="utf-8"))
    return _SOURCES[name]


def _region(name: str, anchor: str) -> str:
    text = source(name)
    at = text.find(anchor)
    assert at >= 0, f"core/src/{name}: no {anchor!r} (update PARSERS in this test)"
    return _block(text, at)


def _arguments(text: str, open_at: int) -> tuple[list[str], int]:
    """The top-level arguments of the call whose "(" is at `open_at`, and the index after its ")"."""
    args, depth, begin, i, n = [], 0, open_at + 1, open_at, len(text)
    while i < n:
        c = text[i]
        if c in "\"'":
            i += 1
            while i < n and text[i] != c:
                i += 2 if text[i] == "\\" else 1
        elif c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
            if depth == 0:
                args.append(text[begin:i].strip())
                return [a for a in args if a], i + 1
        elif c == "," and depth == 1:
            args.append(text[begin:i].strip())
            begin = i + 1
        i += 1
    return [], n


def _parameters(text: str) -> list[str]:
    """The names of the JSON parameters by position ("" for a parameter of another type)."""
    out, depth, begin = [], 0, 0
    for i, c in enumerate(text + ","):
        if c == "<":
            depth += 1
        elif c == ">":
            depth -= 1
        elif c == "," and depth == 0:
            m = re.fullmatch(r"\s*const Json\s*[&*]\s*(\w+)\s*", text[begin:i])
            out.append(m.group(1) if m else "")
            begin = i + 1
    return out


_DEFINITIONS: dict[str, list[tuple[str, list[str], int]]] = {}  # name: [(file, JSON parameters, start)]


def _definitions(file: str, function: str) -> list[tuple[list[str], str]]:
    """The JSON parameters (by position) and the body of every definition of `function`: in `file`, else in any file of
    core/src."""
    if not _DEFINITIONS:
        pattern = re.compile(r"^[A-Za-z_][\w:<>,*& ]*[\s*&](\w+)\(([^)]*)\)\s*(?:const\s*)?\{", re.MULTILINE)
        for path in sorted(SRC.glob("*.cpp")):
            text = source(path.name)
            for m in pattern.finditer(text):
                _DEFINITIONS.setdefault(m.group(1), []).append((path.name, _parameters(m.group(2)), m.start()))
    found = _DEFINITIONS.get(function, [])
    here = [d for d in found if d[0] == file] or found
    return [(params, _block(source(name), at)) for name, params, at in here]


def _of(expression: str, names: set[str]) -> bool:
    """Whether `expression` is (a part of) one of the objects `names`: `j`, `*j`, `j.as_array()[1]`, `arg(j, "k")`."""
    m = re.match(r"[*&\s]*(\w+)\s*(\()?\s*\*?(\w+)?", expression)
    if m is None:
        return False
    if m.group(2):  # a call: of the request when it reads a part of it (arg(j, "k"))
        return m.group(1) not in NOT_A_KEY_READ and m.group(3) in names and "," in expression
    return m.group(1) in names


def aliases(region: str, names: set[str]) -> set[str]:
    """`names` with the local names a part of those objects gets in `region`: `const Json& x = arg(j, "k")`,
    `const Json* x = j.find("k")`, `for (const Json& x : j.as_array())`."""
    names = set(names)
    while True:
        more = set()
        for m in re.finditer(r"\bconst Json\s*[&*]\s*(\w+)\s*=\s*([^;]*);", region):
            if _of(m.group(2), names):
                more.add(m.group(1))
        for m in re.finditer(r"\bfor\s*\(\s*const (?:Json|auto)\s*&\s*(\w+)\s*:\s*([^)]*\)?)\s*\)", region):
            if _of(m.group(2), names):
                more.add(m.group(1))
        if more <= names:
            return names
        names |= more


def keys_read(file: str, region: str, names: set[str] | None, seen: frozenset[str] = frozenset()) -> set[str]:
    """The key literals `region` of `file` reads from the objects `names` (None: from any object), the parts of them it
    names locally and the helpers it hands them to included."""
    if names is not None:
        names = aliases(region, names)

    def of_request(name: str) -> bool:
        return names is None or name in names

    keys: set[str] = set()
    for m in re.finditer(r"\bonly\(\s*(\w+)\s*,\s*\{([^}]*)\}", region):  # the keys a parser takes, others refused
        if of_request(m.group(1)):
            keys |= set(re.findall(rf'"({KEY})"', m.group(2)))
    for m in re.finditer(rf'\b(\w+)\(\s*\*?(\w+)\s*,\s*"({KEY})"', region):  # number_or(j, "key", ...)
        if m.group(1) not in NOT_A_KEY_READ and of_request(m.group(2)):
            keys.add(m.group(3))
    for m in re.finditer(rf'\b(\w+)\s*(?:\.|->)\s*find\(\s*"({KEY})"\s*\)', region):  # j.find("key")
        if of_request(m.group(1)):
            keys.add(m.group(2))
    keys |= set(re.findall(rf'\b(?:k|key|kind|kv\.first)\s*==\s*"({KEY})"', region))  # a loop over the keys
    keys |= set(re.findall(rf'\b(?:number|integer|flag)\(\s*"({KEY})"\s*\)', region))  # a lambda on the object
    if names is None:
        return keys
    for m in re.finditer(r"\b(\w+)\s*\(", region):  # a helper handed the request or a part of it (by position)
        function = m.group(1)
        if function in seen:
            continue
        given = [i for i, a in enumerate(_arguments(region, m.end() - 1)[0]) if _of(a, names)]
        if not given:
            continue
        for params, body in _definitions(file, function):
            inner = {params[i] for i in given if i < len(params) and params[i]}
            if inner:
                keys |= keys_read(file, body, inner, seen | {function})
    return keys


def parser_keys() -> dict[str, set[str]]:
    """The keys each request's parser reads now, by the labels of core/abi/json_keys.json "requests"."""
    out: dict[str, set[str]] = {}
    for function, places in PARSERS.items():
        out[function] = set()
        for file, anchor, names in places:
            out[function] |= keys_read(file, _region(file, anchor), names)
    calls = source(CALL_FILE)
    table = _region(CALL_FILE, "const std::map<std::string, Function>& functions()")
    out["iad_call_json(functions)"] = set()
    for m in re.finditer(r'\{"(\w+\.\w+)",', table):
        entry = _block(calls, calls.find(m.group(0), calls.find("Function>& functions()")))
        lam = re.search(r"\[\]\(const Json&\s*(\w*)\)", entry)  # [](const Json&) takes no argument
        names = {lam.group(1)} if lam and lam.group(1) else set()
        out[f"iad_call_json({m.group(1)})"] = keys_read(CALL_FILE, entry, names)
    info = _region(INFO_FILE, "Json info_call(Axis& axis, const std::string& function, const Json& args)")
    for m in re.finditer(r'if \(function == "(\w+)"\)', info):
        out[f"iad_axis_info_call_json({m.group(1)})"] = keys_read(INFO_FILE, _block(info, m.start()), {"args"})
    return out


# ====================================================================== reading the header


def header_requests(header: str) -> dict[str, str]:
    """{function: its declaration} of every stable function that takes a JSON text."""
    out = {}
    for m in re.finditer(r"IAD_API iad_result IAD_CALL (\w+)\(([^;]*)\);", header):
        if re.search(r"const char\s*\*\s*(request|\w+_json)\b", m.group(2)):
            out[m.group(1)] = m.group(0)
    return out


def header_comment(header: str, function: str) -> str:
    """The comment of `function` in the header: its own /** ... */ block right above its declaration; for a function
    whose request the section documents (SECTION_DOCUMENTED), from the section's heading to the declaration."""
    at = header.find(f"IAD_CALL {function}(")
    assert at >= 0, f"{function} is not in the header"
    if function in SECTION_DOCUMENTED:
        return header[header.rfind("/* ---", 0, at):at]
    begin = header.rfind("/**", 0, at)
    end = header.find("*/", begin) + 2
    assert begin >= 0 and re.fullmatch(r"\s*IAD_API iad_result ", header[end:at]), f"{function} has no /** comment"
    return header[begin:end]


def _fields(outputs: dict[str, dict[str, str]], output: str, top_only: bool) -> set[str]:
    names = set()
    for path in outputs[output]:
        parts = re.findall(r"\.(\w+)", path)
        if parts and (not top_only or len(parts) == 1):
            names.add(parts[-1])
    return names


def problems(committed: dict, now: dict[str, set[str]], header: str) -> list[str]:
    """What the committed "requests" (and the header comment) say against what the parsers read."""
    requests, outputs = committed["requests"], committed["outputs"]
    out = []
    declared = set()
    for function in header_requests(header):
        if function in ("iad_call_json", "iad_axis_info_call_json"):
            declared |= {label for label in now if label.startswith(function + "(")}
        else:
            declared.add(function)
    for label in sorted(declared - set(requests)):
        out.append(f"{label}: takes JSON but core/abi/json_keys.json \"requests\" does not list it")
    for label in sorted(set(requests) - declared):
        out.append(f"{label}: listed in \"requests\" but the header has no such JSON-taking function")
    for label in sorted(declared - set(now)):
        out.append(f"{label}: no parser for it in PARSERS of this test")
    for label in sorted(declared & set(requests) & set(now)):
        listed, read = requests[label], now[label]
        if isinstance(listed, str):  # a document: the keys read are fields of that output
            for key in sorted(read - _fields(outputs, listed, top_only=False)):
                out.append(f"{label}: reads '{key}', which is no field of {listed}")
            continue
        for key in sorted(read - set(listed)):
            out.append(f"{label}: the parser reads '{key}', which \"requests\" does not list")
        for key in sorted(set(listed) - read):
            out.append(f"{label}: \"requests\" lists '{key}', which the parser does not read")
        if label.startswith("iad_call_json("):
            continue
        function = label.split("(")[0]
        if function in FIELDS_OF:
            for key in sorted(set(listed) - _fields(outputs, FIELDS_OF[function], top_only=True)):
                out.append(f"{label}: '{key}' is no field of {FIELDS_OF[function]} (as the header says)")
            continue
        comment = header_comment(header, function)
        for key in sorted(listed):
            if f'"{key}"' not in comment:
                out.append(f"{label}: the header comment does not name \"{key}\"")
    return out


# ====================================================================== the test


def _ready() -> str | None:
    if not (KEYS_FILE.is_file() and HEADER.is_file() and SRC.is_dir()):
        return "core/abi/json_keys.json and core/src are in a source checkout only"
    return None


SKIP = _ready()


@unittest.skipIf(SKIP is not None, SKIP or "")
class RequestKeysTest(unittest.TestCase):
    """Every key a parser reads is listed (and named by the header), and every listed key is read."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.committed = json.loads(KEYS_FILE.read_text(encoding="utf-8"))
        cls.header = HEADER.read_text(encoding="utf-8")
        cls.now = parser_keys()

    def test_the_request_keys_are_listed_and_named(self) -> None:
        self.assertEqual(problems(self.committed, self.now, self.header), [])

    def test_the_coordinated_request_takes_expect_start(self) -> None:
        for function in ("iad_cell_coord_plan_json", "iad_cell_coord_run_json"):
            self.assertIn("expect_start", self.now[function])
            self.assertIn("expect_start", self.committed["requests"][function])
        self.assertIn('"expect_start"', header_comment(self.header, "iad_cell_coord_run_json"))

    def test_the_comparison_catches_a_missing_and_an_extra_key(self) -> None:
        requests = dict(self.committed["requests"])
        label = "iad_bus_state_line_rules"
        requests[label] = [k for k in requests[label] if k != "coils"] + ["no_such_key"]
        found = problems({**self.committed, "requests": requests}, self.now, self.header)
        self.assertEqual(len([p for p in found if p.startswith(label + ":")]), 3, found)  # coils, no_such_key twice
        header = self.header.replace('"coils": false, ', "")
        found = problems(self.committed, self.now, header)
        self.assertEqual(found, [f'{label}: the header comment does not name "coils"'])

    def test_the_header_comment_is_the_function_s_own(self) -> None:
        comment = header_comment(self.header, "iad_bus_state_line_rules")
        self.assertTrue(comment.startswith("/**") and comment.endswith("*/"), comment)
        self.assertNotIn("/* ---", comment)
        self.assertIn("/* ---", header_comment(self.header, "iad_sim_open"))  # documented by its section

    def test_the_scan(self) -> None:
        region = 'void f(const Json& j) { only(j, {"a", "b"}); g(j); x = j.find("c"); y = opt(q, "nested"); }'
        self.assertEqual(keys_read(CALL_FILE, region, {"j"}), {"a", "b", "c"})
        # a part under a local name, a loop over it, and a helper handed a part (unit_arg reads the unit's keys)
        region = ('void f(const Json& a) { const Json& t = arg(a, "table"); for (const Json& d : t.as_array()) '
                  'b = need_int32(d, "bit"); const Json* m = a.find("mode"); c = m->find("code"); '
                  'u = unit_arg(a, "unit"); }')
        found = keys_read(CALL_FILE, region, {"a"})
        self.assertEqual(found - {"max_speed", "max_accel", "max_push_speed", "lead_mm", "encoder_pulses",
                                  "min_band_mm", "rotary"}, {"table", "bit", "mode", "code", "unit"})
        self.assertIn("encoder_pulses", found)
        self.assertEqual(_blank_comments('a // "k"\nb /* "k" */ "s//t"'), 'a       \nb           "s//t"')


if __name__ == "__main__":
    unittest.main()
