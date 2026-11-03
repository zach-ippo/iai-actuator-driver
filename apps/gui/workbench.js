// Axis Workbench: every connected axis, every driver function, the unit's own ranges.
// Talks only to the local server (/api/wb/*). STOP ALL: header button, Space or Esc.
"use strict";

const TOKEN = window.ECToken.get(); // the access key from the launch address (token.js)
const $ = (id) => document.getElementById(id);

let CATALOG = null; // {axis: [...], port: [...], group: [...], sim_setups: [...], nv: {...}}
let STATE = { ports: [], axes: [] };
let SEL = null; // selected axis name
let lastSeq = 0;
let renderedFor = { op: null, fn: null, group: null }; // axis name + limits signature the forms were built for
let paramsCache = [];
let recovering = null; // axis name while its Recover request runs
const stateListeners = []; // WB.onState
const viewListeners = []; // WB.onView
const clientErrors = []; // {t, message}: error toasts and page errors, sent with the test report
const selfChecks = {}; // axis name -> the last self-test run from this page (with its items)
const VIEWS = ["viewAxis", "viewGroup", "viewFunctions", "viewCoord"];
const VIEW_NAMES = { viewAxis: "Axis", viewGroup: "Group", viewFunctions: "Functions", viewCoord: "Coordinated" };
const KEY_START = "ecgui.startHere.hidden";
const KEY_API = "ecgui.showApi";
const KEY_VIEW = "ecgui.wb.view";

// ---------------------------------------------------------------- helpers

// Browser storage is a convenience only: private windows or blocked site data make it throw.
function stored(key) {
  try {
    return window.localStorage.getItem(key);
  } catch {
    return null;
  }
}
function store(key, value) {
  try {
    window.localStorage.setItem(key, value);
  } catch {
    // no storage: the page works without it
  }
}

function h(tag, attrs, ...kids) {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (v === undefined || v === null || v === false) continue;
    if (k === "class") el.className = v;
    else if (k === "text") el.textContent = v;
    else if (k === "title") el.title = v;
    else if (k.startsWith("on")) el.addEventListener(k.slice(2), v);
    else if (k === "dataset") Object.assign(el.dataset, v);
    else if (v === true) el.setAttribute(k, "");
    else el.setAttribute(k, String(v));
  }
  for (const kid of kids.flat()) {
    if (kid === null || kid === undefined || kid === false) continue;
    el.append(kid instanceof Node ? kid : document.createTextNode(String(kid)));
  }
  return el;
}

function httpError(message, status, data) {
  const e = Error(message);
  e.status = status;
  e.kind = (data && data.kind) || null;
  e.data = data || null;
  return e;
}

async function rawApi(path, data, signal) {
  let r;
  try {
    r = await fetch(path, {
      method: data === undefined ? "GET" : "POST",
      headers: { "X-EC-Token": TOKEN, "Content-Type": "application/json" },
      body: data === undefined ? undefined : JSON.stringify(data),
      signal,
    });
  } catch (e) {
    if (e && e.name === "AbortError") throw httpError("no answer in time", 0, null);
    throw httpError(`Server not reachable (${e.message})`, 0, null);
  }
  let v;
  try {
    v = await r.json();
  } catch {
    throw httpError(`Server returned ${r.status} without JSON`, r.status, null);
  }
  if (!r.ok || v.ok === false) throw httpError(v.error || `Request failed (${r.status})`, r.status, v);
  return v;
}

// A browser keeps at most 6 connections to one host and the server answers a long request (a port scan, a backup,
// Recover) on its own connection: a page with 6 of them in flight cannot deliver STOP ALL until one ends. So the page
// has a request budget: STOP and the two status polls bypass it; every other request waits for one of BUDGET places.
// That leaves 2 connections for STOP even while both polls run.
const BUDGET = 2;
const BYPASS = ["/api/wb/stop", "/api/wb/coord/stop", "/api/wb/state", "/api/wb/coord/status"];
let inFlight = 0;
const waiting = [];
async function api(path, data) {
  if (BYPASS.includes(path.split("?")[0])) return rawApi(path, data);
  if (inFlight >= BUDGET) await new Promise((go) => waiting.push(go)); // a finishing request hands its place over
  else inFlight += 1;
  try {
    return await rawApi(path, data);
  } finally {
    const next = waiting.shift();
    if (next) next();
    else inFlight -= 1;
  }
}

// Errors seen by the page (toasts, script errors), newest last, at most 30; the test report lists them.
function noteError(message) {
  const m = String(message === undefined || message === null ? "" : message).trim().slice(0, 300);
  if (!m) return;
  const last = clientErrors[clientErrors.length - 1];
  if (last && last.message === m) {
    last.t = Date.now() / 1000;
    return;
  }
  clientErrors.push({ t: Date.now() / 1000, message: m });
  while (clientErrors.length > 30) clientErrors.shift();
}
window.addEventListener("error", (ev) => {
  const where = ev.filename ? ` (${String(ev.filename).split("/").pop()}:${ev.lineno})` : "";
  noteError(`Page error: ${ev.message}${where}`);
});
window.addEventListener("unhandledrejection", (ev) => {
  const r = ev.reason;
  noteError(`Page error: ${r && r.message ? r.message : r}`);
});

// Toasts: errors stay 10 s, confirmations 2.5 s; at most 3 on screen (confirmations make room first).
const TOAST_MAX = 3;
function toast(msg, ok = false) {
  const text = String(msg);
  if (!ok) noteError(text);
  const box = $("toasts");
  if (!box) return;
  let timer = null;
  const close = h("button", { type: "button", class: "toast-close", title: "Close", "aria-label": "Close", text: "×" });
  const t = h("div", { class: ok ? "toast ok" : "toast", role: ok ? "status" : "alert" }, h("div", { class: "toast-msg", text }), close);
  const remove = () => {
    clearTimeout(timer);
    t.remove();
  };
  close.addEventListener("click", remove);
  timer = setTimeout(remove, ok ? 2500 : 10000);
  box.append(t);
  while (box.children.length > TOAST_MAX) (box.querySelector(".toast.ok") || box.firstElementChild).remove();
}

const num = (v) => (v === null || v === undefined || v === "" ? null : Number(v));
function fmtN(v, digits = 2) {
  if (v === null || v === undefined || Number.isNaN(Number(v))) return "?";
  return String(Number(Number(v).toFixed(digits)));
}
const axisOf = (name) => STATE.axes.find((a) => a.name === name) || null;
const selAxis = () => axisOf(SEL);
const unitsOf = (ax) => (ax && ax.limits && ax.limits.units) || "mm";

// The page interface other scripts use (coord.js). Assigned before anything else runs.
window.WB = {
  api,
  h,
  toast,
  ask,
  fmtN,
  noteError,
  stopAll: () => stopAll(),
  poll: () => poll(),
  state: () => STATE,
  catalog: () => CATALOG,
  axisOf: (name) => axisOf(name),
  unitsOf: (ax) => unitsOf(ax),
  onState: (fn) => {
    if (typeof fn === "function") stateListeners.push(fn);
  },
  onView: (fn) => {
    if (typeof fn === "function") viewListeners.push(fn);
  },
  currentView: () => currentView(),
  showView: (id) => showView(id),
  addSimPair: () => addSimPair(),
};

// ---------------------------------------------------------------- confirm dialog

// One plain OK / Cancel before a write: no tick box keeps OK disabled.
function ask({ title, text, okText = "OK", danger = false }) {
  const dlg = $("confirmDlg");
  $("dlgTitle").textContent = title;
  $("dlgText").textContent = text;
  $("dlgOk").textContent = okText;
  $("dlgOk").className = danger ? "stop-mini" : "primary";
  return new Promise((resolve) => {
    const done = () => {
      dlg.removeEventListener("close", done);
      resolve(dlg.returnValue === "ok");
    };
    dlg.addEventListener("close", done);
    dlg.returnValue = "cancel";
    dlg.showModal();
  });
}

// ---------------------------------------------------------------- ranges and inputs

function rangeFor(inp, limits) {
  if (inp.range) {
    const r = (limits || {})[inp.range];
    if (!r) return null;
    return { min: r.min, max: r.max, unit: r.unit || "", source: r.source || "", available: r.available };
  }
  if (inp.min !== undefined) return { min: inp.min, max: inp.max, unit: "", source: "" };
  return null;
}

function rangeText(r, inp) {
  if (!r) return "";
  if (r.available === false) return "not available on this unit";
  const hex = inp.type === "int" && typeof inp.example === "string" && inp.example.startsWith("0x");
  const f = (v) => (hex && v !== null && v !== undefined ? "0x" + Number(v).toString(16).toUpperCase() : fmtN(v));
  return `${f(r.min)} – ${f(r.max)}${r.unit ? " " + r.unit : ""}`;
}

function exampleFor(inp, r) {
  const ex = inp.example;
  if (typeof ex === "string" && ex.startsWith("frac:")) {
    if (!r || r.min === null || r.max === null || r.min === undefined || r.max === undefined) return "";
    return fmtN(r.min + (r.max - r.min) * parseFloat(ex.slice(5)));
  }
  if (typeof ex === "string" && ex.startsWith("cap:")) {
    const v = parseFloat(ex.slice(4));
    return fmtN(r && r.max !== null && r.max !== undefined ? Math.min(v, r.max) : v);
  }
  return ex === undefined || ex === null ? "" : String(ex);
}

function parseNumber(text, isInt) {
  const t = String(text).trim();
  if (isInt && /^0x[0-9a-f]+$/i.test(t)) return parseInt(t, 16);
  if (!/^[-+]?(\d+\.?\d*|\.\d+)(e[-+]?\d+)?$/i.test(t)) return NaN;
  return Number(t);
}

// One input control: returns {el, get, set}. get() returns the value or throws with the allowed range.
function buildInput(inp, limits, { prefill = true, compact = false } = {}) {
  const r = rangeFor(inp, limits);
  const label = inp.label;
  if (inp.type === "bool") {
    const box = h("input", { type: "checkbox", name: inp.name });
    box.checked = prefill && inp.example === true;
    const el = h("label", { class: "check" }, box, label);
    return { el, get: () => box.checked, set: (v) => (box.checked = !!v) };
  }
  if (inp.type === "choice") {
    const sel = h("select", { name: inp.name },
      (inp.choices || []).map(([v, t]) => h("option", { value: String(v), text: t })));
    if (prefill && inp.example !== undefined) sel.value = String(inp.example);
    const el = h("label", {}, label, sel);
    const get = () => {
      const c = (inp.choices || []).find(([v]) => String(v) === sel.value);
      if (!c || c[0] === "") return undefined;
      return c[0];
    };
    return { el, get, set: (v) => (sel.value = String(v)) };
  }
  const example = exampleFor(inp, r);
  const isNum = inp.type === "number" || inp.type === "int";
  const field = h("input", {
    type: "text", name: inp.name, autocomplete: "off", spellcheck: "false",
    inputmode: isNum ? "decimal" : undefined,
    placeholder: example !== "" ? `e.g. ${example}` : inp.optional ? "optional" : "",
  });
  if (prefill && !inp.optional && example !== "") field.value = example;
  const hint = h("span", { class: "range", text: isNum ? rangeText(r, inp) || "any number" : inp.optional ? "optional" : "" });
  if (r && r.source) hint.title = `Range source: ${r.source}`;
  const el = h("label", { title: r && r.source ? `${label}: ${rangeText(r, inp)} (${r.source})` : label },
    compact ? null : label, field, hint);
  const check = () => {
    const t = field.value.trim();
    if (t === "") {
      if (inp.optional) return undefined;
      throw Error(`${label} is required${r ? ` (${rangeText(r, inp)})` : ""}.`);
    }
    if (!isNum) return t;
    const v = parseNumber(t, inp.type === "int");
    if (Number.isNaN(v)) throw Error(`${label}: "${t}" is not a number.`);
    if (inp.type === "int" && !Number.isInteger(v)) throw Error(`${label} must be a whole number.`);
    if (r && r.available === false && v !== 0) throw Error(`${label}: not available on this unit.`);
    if (r && r.min !== null && r.min !== undefined && v < r.min - 1e-9) throw Error(`${label} must be ${rangeText(r, inp)} (got ${t}).`);
    if (r && r.max !== null && r.max !== undefined && v > r.max + 1e-9) throw Error(`${label} must be ${rangeText(r, inp)} (got ${t}).`);
    return v;
  };
  field.addEventListener("input", () => {
    let bad = false;
    try {
      check();
    } catch {
      bad = field.value.trim() !== "";
    }
    field.classList.toggle("invalid", bad);
    hint.classList.toggle("bad", bad);
  });
  return { el, get: check, set: (v) => (field.value = v === null || v === undefined ? "" : String(v)), field };
}

// ---------------------------------------------------------------- non-volatile writes

const nvInfo = () => (CATALOG && CATALOG.nv) || { cycles: 100000, warn_per_hour: 60, skip_note: "unchanged: nothing written" };

// "unchanged: nothing written" for a write the driver skipped, else "written and verified".
function nvResultText(r) {
  if (r && r.skipped) return r.note || nvInfo().skip_note || "unchanged: nothing written";
  return "written and verified";
}

// The line the non-volatile confirm dialogs add (empty while the server does not count writes).
function nvCountLine(axisName) {
  const ax = axisOf(axisName);
  return ax && typeof ax.nv_writes === "number" ? `\n\nNon-volatile writes on this axis in this session: ${ax.nv_writes}.` : "";
}

function renderNvNotes() {
  const ax = selAxis();
  const nv = nvInfo();
  const cycles = Number(nv.cycles || 100000).toLocaleString("en-US");
  let text = `Non-volatile memory lasts about ${cycles} write cycles. Unchanged values are not written.`;
  let many = false;
  if (ax && typeof ax.nv_writes === "number") {
    const hour = typeof ax.nv_writes_last_hour === "number" ? ax.nv_writes_last_hour : null;
    text += ` This axis: ${ax.nv_writes} write${ax.nv_writes === 1 ? "" : "s"} in this session${hour === null ? "" : ` (${hour} in the last hour)`}.`;
    many = hour !== null && !!nv.warn_per_hour && hour > nv.warn_per_hour;
    if (many) text += ` More than ${nv.warn_per_hour} in the last hour: check that nothing writes in a loop.`;
  }
  for (const id of ["posNvNote", "paramNvNote"]) {
    const p = $(id);
    if (p.textContent !== text) p.textContent = text;
    p.classList.toggle("warn", many);
  }
}

// ---------------------------------------------------------------- function cards

// A function card built from the catalogue: description, inputs with ranges / examples, Run, result.
// Operating panels (operating: true) show the plain description; the Python call and the evidence notes go behind
// the API toggle. The Functions view (the API explorer) shows the full text.
function fnCard(spec, { axis, showResult = true, compact = false, extra = null, portFn = false, operating = false } = {}) {
  const ax = axisOf(axis);
  const limits = (ax && ax.limits) || {};
  const controls = {};
  const inputs = h("div", { class: "fn-inputs" });
  for (const inp of spec.inputs || []) {
    const c = buildInput(inp, limits);
    controls[inp.name] = c;
    inputs.append(c.el);
  }
  const msg = h("span", { class: "fn-msg" });
  const out = h("pre", { class: "json hidden" });
  const run = h("button", { type: "button", class: spec.name === "stop" ? "stop-mini" : "primary", text: compact ? spec.label : "Run" });
  const evidence = operating && (spec.evidence || []).length
    ? h("p", { class: "fn-evidence api-note", text: `Evidence: ${spec.evidence.join("; ")}` }) : null;
  const card = h("div", { class: "card fn-card", dataset: { fn: spec.name } },
    h("h3", { text: spec.label }),
    h("div", { class: operating ? "fn-call api-note" : "fn-call", text: spec.call }),
    h("p", { class: "fn-desc", text: operating ? spec.plain || spec.desc : spec.desc }),
    evidence,
    (spec.inputs || []).length ? inputs : null,
    extra,
    h("div", { class: "fn-actions" }, run, msg),
    showResult ? out : null);
  const gather = () => {
    const args = {};
    for (const [k, c] of Object.entries(controls)) {
      const v = c.get();
      if (v !== undefined) args[k] = v;
    }
    return args;
  };
  const exec = async (overrides = {}) => {
    msg.className = "fn-msg";
    msg.textContent = "";
    let args;
    try {
      args = { ...gather(), ...overrides };
    } catch (e) {
      msg.className = "fn-msg err";
      msg.textContent = e.message;
      return;
    }
    if (!(await confirmFor(spec, args, axis))) return;
    run.disabled = true;
    msg.textContent = "running...";
    const t0 = performance.now();
    try {
      const v = portFn
        ? await api("/api/wb/port", { port: ax ? ax.port : null, fn: spec.name, args })
        : await api("/api/wb/call", { axis, fn: spec.name, args });
      msg.className = "fn-msg ok";
      msg.textContent = `ok · ${Math.round(performance.now() - t0)} ms`;
      if (spec.confirm === "nonvolatile") msg.textContent += ` · ${nvResultText(v.result)}`;
      if (showResult) {
        out.textContent = JSON.stringify(v.result !== undefined ? v.result : v, null, 2);
        out.classList.remove("hidden");
      }
      if (["write_parameter", "software_reset", "write_position_table", "teach_position", "set_baudrate", "scan"].includes(spec.name)) poll();
    } catch (e) {
      msg.className = "fn-msg err";
      msg.textContent = e.message;
      noteError(`${axis || "-"}: ${spec.label}: ${e.message}`);
      if (showResult) out.classList.add("hidden");
    } finally {
      run.disabled = false;
    }
  };
  run.addEventListener("click", () => exec());
  card.exec = exec;
  card.controls = controls;
  return card;
}

async function confirmFor(spec, args, axis) {
  if (spec.confirm === "nonvolatile") {
    return ask({
      title: `${spec.label} on ${axis}?`,
      text: `${spec.plain || spec.desc}\n\nThis writes the unit's NON-VOLATILE memory (PERMANENT until changed again).\n` +
        `Arguments: ${JSON.stringify(args)}\n\nBack up the unit first (Functions > Back up this unit).` + nvCountLine(axis),
      okText: "Write",
    });
  }
  if (spec.confirm === "reset") {
    return ask({ title: `Software reset ${axis}?`, text: "SOFTWARE RESET: the unit switches the servo off and restarts its firmware. The axis must be at rest.", okText: "Reset" });
  }
  if (spec.confirm === "danger") {
    return ask({
      title: `${spec.label}?`, text: `${spec.plain || spec.desc}\nArguments: ${JSON.stringify(args)}` +
        "\n\nEvery unit on this port switches its baud rate.",
      okText: "Change", danger: true,
    });
  }
  return true;
}

// ---------------------------------------------------------------- lamps

const LAMPS = [
  ["SV", "servo_on", "ok", "Servo on"],
  ["HOME", "homed", "ok", "Home return complete (homing: blinking)"],
  ["INP", "in_position", "ok", "In position (PEND)"],
  ["MOVE", "moving", "busy", "Moving"],
  ["PUSH", "pushing", "busy", "Pushing (contact: steady, miss: amber)"],
  ["PAUSE", "paused", "warn", "Paused (STP held)"],
  ["WARN", "warning", "warn", "Warning-level alarm"],
  ["ALM", "alarm", "alert", "Alarm"],
  ["EMG", "emergency_stop", "alert", "Emergency stop"],
  ["BWD", "at_backward_end", "marker", "At the backward end (LS0)"],
  ["FWD", "at_forward_end", "marker", "At the forward end (LS1)"],
  ["MID", "at_middle", "marker", "At the middle position (LS2)"],
  ["PC", "modbus_enabled", "ok", "Modbus (PC) control taken (PMSS)"],
];

// Condition flags of the 22-word status (DOC ME0162-12A p.4-18 to 4-25), shown only while they are set; null = the unit
// does not define the bit (RCON) or the word was not read.
const CONDITIONS = [
  ["EMG IN", "emg_input", "alert", "Emergency-stop input active"],
  ["PWR CUT", "drive_power_cut", "alert", "Motor drive power cut (MPOW = 0 / MPUV)"],
  ["ENB OFF", "enable_off", "warn", "Teaching-tool enable switch released: the servo is held off"],
  ["COLD", "cold_start_alarm", "alert", "A cold-start-level alarm is present: software reset or power cycle"],
  ["ASOF", "auto_servo_off", "warn", "Automatic servo OFF: the next move switches the servo back on"],
  ["NV", "nv_busy", "busy", "Non-volatile memory write in progress"],
  ["AUTO", "auto_mode", "warn", "AUTO mode: the PIO has control"],
  ["BATT", "battery_low", "warn", "Absolute-encoder battery voltage low"],
  ["EXC", "exciting", "busy", "Excitation detection running"],
];

function conditionLamps(st) {
  if (!st) return [];
  const on = { ...st, auto_mode: st.manual_mode === false, exciting: !!st.servo_on && st.excitation_done === false };
  return CONDITIONS.filter(([, key]) => on[key]).map(([label, , kind, title]) => h("span", { class: `ml on ${kind}`, title }, h("i"), label));
}

function lampsFor(st) {
  return LAMPS.map(([label, key, kind, title]) => {
    let on = !!(st && st[key]);
    let k = kind;
    if (key === "homed" && st && st.homing) { on = true; k = "busy"; }
    if (key === "pushing" && st) {
      if (st.push_contact) { on = true; k = "ok"; }
      else if (st.push_missed) { on = true; k = "warn"; }
    }
    const unknown = st && st[key] === null && kind === "marker";
    if (unknown) return null;
    const text = key === "alarm" && on && st.alarm_code ? `ALM ${st.alarm_code.toString(16).toUpperCase().padStart(3, "0")}` : label;
    return h("span", { class: `ml ${on ? "on " + k : ""}`, title }, h("i"), text);
  });
}

// ---------------------------------------------------------------- polling and rendering

// poll() always reads a fresh state: a call while one is running waits for it and reads again.
let pollRunning = null;
let pollQueued = null;
let serverDown = false;
function poll() {
  if (!pollRunning) {
    pollRunning = pollOnce().finally(() => {
      pollRunning = null;
    });
    return pollRunning;
  }
  if (!pollQueued) {
    pollQueued = pollRunning.then(() => {
      pollQueued = null;
      return poll();
    });
  }
  return pollQueued;
}

async function pollOnce() {
  let v;
  try {
    v = await api(`/api/wb/state?since=${lastSeq}`);
  } catch (e) {
    $("connBadge").textContent = "Server not reachable";
    $("connBadge").className = "badge bad";
    if (!serverDown) noteError(`Server not reachable: ${e.message}`);
    serverDown = true;
    return;
  }
  serverDown = false;
  STATE = v;
  for (const e of v.events) addEvent(e);
  lastSeq = v.seq;
  if (SEL && !axisOf(SEL)) SEL = null;
  if (!SEL && v.axes.length) selectAxis(v.axes[0].name);
  render();
}

function addEvent(e) {
  const list = $("events");
  const time = new Date(e.t * 1000).toLocaleTimeString();
  list.prepend(h("li", { class: e.level === "info" ? "" : e.level }, h("time", { text: time }), e.message));
  while (list.children.length > 150) list.lastChild.remove();
}

function render() {
  const n = STATE.axes.length;
  const alarms = STATE.axes.filter((a) => a.status && a.status.alarm).length;
  $("connBadge").textContent = n ? `${n} ax${n === 1 ? "is" : "es"} · ${STATE.ports.length} port${STATE.ports.length === 1 ? "" : "s"}${alarms ? ` · ${alarms} alarm` : ""}` : "No axis";
  $("connBadge").className = n ? (alarms ? "badge bad" : "badge") : "badge off";
  updateEmpties();
  renderConnected();
  renderCards();
  renderHead();
  maybeBuildForms();
  renderGroupLive();
  renderNvNotes();
  renderSelfCheck(selAxis());
  for (const fn of stateListeners) {
    try {
      fn(STATE);
    } catch (e) {
      noteError(`Page error: ${e.message}`);
    }
  }
}

// Empty states: the general one on the Axis and Functions views; the Group view has its own (two axes needed);
// coord.js draws the Coordinated one.
function updateEmpties() {
  const n = STATE.axes.length;
  const view = currentView();
  $("emptyState").classList.toggle("hidden", n > 0 || view === "viewGroup" || view === "viewCoord");
  $("groupEmpty").classList.toggle("hidden", n >= 2);
  $("groupCard").classList.toggle("hidden", n === 0);
}

function baudText(p) {
  if (!p.baud) return "no baud";
  const how = p.simulated ? "" : p.baud_detected === true ? " (detected)" : p.baud_detected === false ? " (set)" : "";
  return `${p.baud} baud${how}`;
}

function renderConnected() {
  const ul = $("connectedPorts");
  const sig = JSON.stringify(STATE.ports);
  if (ul.dataset.sig === sig) return;
  ul.dataset.sig = sig;
  ul.replaceChildren();
  if (!STATE.ports.length) ul.append(h("li", { class: "small", text: "None yet." }));
  for (const p of STATE.ports) {
    const n = p.axes.length;
    const line = [`${n} ax${n === 1 ? "is" : "es"}`, baudText(p), p.line && p.line !== "direct" ? `${p.line} line` : null].filter(Boolean).join(" · ");
    ul.append(h("li", {},
      h("div", { class: "grow" },
        h("b", { text: p.name }), " ",
        p.simulated ? h("span", { class: "tag sim", text: "sim" }) : null,
        p.is_iai ? h("span", { class: "tag", text: "IAI" }) : null,
        h("div", { class: "small", text: line })),
      h("button", { type: "button", class: "mini", text: "Scan", title: "Scan this port again for more axes",
        onclick: () => scanPort(p.name) }),
      h("button", { type: "button", class: "mini", text: "Disconnect",
        title: "Stop moving axes, release HOME, hand control back to the PIO where it had it, then close the port. The servo is left as it is.",
        onclick: () => disconnectPort(p.name) })));
  }
  loadPorts(false);
}

// An axis whose status is older than STALE_S shows it: while an operation holds the axis (a coordinated plan, a back-up,
// the unit info, a parameter read) the poll skips it, and an old position or "moving" flag must not pass for the
// unit's state now.
const STALE_S = 1.0;
const isStale = (ax) => ax.age_s !== null && ax.age_s !== undefined && ax.age_s > STALE_S;
const ageText = (ax) => `status ${ax.age_s >= 10 ? Math.round(ax.age_s) : ax.age_s.toFixed(1)} s old`;

function renderCards() {
  const box = $("axisCards");
  const names = STATE.axes.map((a) => a.name);
  for (const el of [...box.children]) if (!names.includes(el.dataset.axis)) el.remove();
  for (const ax of STATE.axes) {
    let card = box.querySelector(`[data-axis="${CSS.escape(ax.name)}"]`);
    if (!card) {
      card = h("div", { class: "axis-card", role: "option", tabindex: "0", dataset: { axis: ax.name },
        onclick: () => selectAxis(ax.name),
        onkeydown: (ev) => { if (ev.key === "Enter") selectAxis(ax.name); } },
        h("div", { class: "ac-head" }, h("span", { class: "ac-name" }), h("span", { class: "tag ac-kind" })),
        h("div", { class: "ac-model" }),
        h("div", { class: "ac-pos" }),
        h("div", { class: "mini-lamps" }),
        h("div", { class: "small ac-busy" }),
        h("button", { type: "button", class: "ac-stop", text: "STOP", title: `Stop ${ax.name}`,
          onclick: (ev) => { ev.stopPropagation(); stopAxis(ax.name); } }));
      box.append(card);
    }
    const st = ax.status || {};
    card.classList.toggle("selected", ax.name === SEL);
    card.classList.toggle("alarm", !!st.alarm);
    card.classList.toggle("fault", !!ax.fault);
    card.setAttribute("aria-selected", ax.name === SEL ? "true" : "false");
    card.querySelector(".ac-name").textContent = ax.name;
    card.querySelector(".ac-kind").textContent = (ax.limits && ax.limits.kind) || (ax.info && ax.info.kind) || "axis";
    card.querySelector(".ac-model").textContent = (ax.info && (ax.info.model || ax.info.controller)) || "unknown model";
    card.querySelector(".ac-pos").replaceChildren(st.position_mm === undefined || st.position_mm === null ? "–" : fmtN(st.position_mm),
      h("small", { text: unitsOf(ax) }));
    card.querySelector(".mini-lamps").replaceChildren(...lampsFor(ax.status).filter(Boolean));
    const stale = isStale(ax);
    card.classList.toggle("stale", stale);
    const busy = ax.fault ? `! ${FAULT_TITLES[ax.fault.kind] || "link fault"}: Recover`
      : ax.error ? `! ${ax.error}` : ax.busy ? `busy: ${ax.busy}` : (ax.fast ? "fast mode" : "");
    card.querySelector(".ac-busy").textContent = stale ? [busy, ageText(ax)].filter(Boolean).join(" · ") : busy;
  }
}

function selectAxis(name) {
  if (SEL === name) return;
  SEL = name;
  renderedFor = { op: null, fn: null, group: renderedFor.group };
  render();
  const pane = currentPane();
  if (pane === "panePositions") loadPositions();
  if (pane === "paneParams") loadParams();
  if (pane === "paneInfo") loadOverview();
}

function renderHead() {
  const ax = selAxis();
  $("axisHead").classList.toggle("hidden", !ax);
  $("axisTabs").classList.toggle("hidden", !ax);
  for (const p of document.querySelectorAll(".pane")) p.classList.toggle("hidden", !ax || p.id !== currentPane());
  $("fnAxis").textContent = `Axis: ${ax ? ax.name : "- (connect one)"}`;
  if (!ax) return;
  const st = ax.status || {};
  const lim = ax.limits || {};
  $("axName").textContent = ax.name;
  const info = ax.info || {};
  $("axModel").textContent = [info.model, info.controller !== info.model ? info.controller : null, lim.kind, lim.family,
    `axis no. ${ax.axis_number}`].filter(Boolean).join(" · ");
  const pos = lim.position || {};
  const u = unitsOf(ax);
  $("travelMin").textContent = `${fmtN(pos.min)} ${u}`;
  $("travelMax").textContent = `${fmtN(pos.max)} ${u}`;
  $("travelPos").textContent = st.position_mm === undefined || st.position_mm === null ? "–" : `${fmtN(st.position_mm)} ${u}`;
  const stale = isStale(ax);
  $("axisHead").classList.toggle("stale", stale);
  $("axStale").classList.toggle("hidden", !stale);
  $("axStale").textContent = stale ? `${ageText(ax)}: the position, speed and flags below are the last ones read, not the unit's state now.` : "";
  const span = (pos.max ?? 0) - (pos.min ?? 0);
  if (span > 0 && st.position_mm !== undefined && st.position_mm !== null) {
    const f = Math.max(0, Math.min(1, (st.position_mm - pos.min) / span));
    $("travelCar").style.left = `${(f * 100).toFixed(2)}%`;
  }
  $("axLamps").replaceChildren(...[...lampsFor(ax.status), ...conditionLamps(ax.status),
    st.velocity_mm_s ? h("span", { class: "ml", text: `${fmtN(st.velocity_mm_s, 1)} ${u}/s` }) : null,
    st.alarm_name ? h("span", { class: "ml on alert", text: st.alarm_name }) : null,
    st.source === "monitor" && ax.age_s !== null && ax.age_s < 1 ? h("span", { class: "ml on", title: "Live status from the axis's status monitor (Fast mode)" }, h("i"), "LIVE") : null].filter(Boolean));
  const err = $("axError");
  err.textContent = ax.error || "";
  err.classList.toggle("hidden", !ax.error);
  const fast = $("fastToggle");
  const mon = ax.monitor && ax.monitor.running ? ax.monitor : null;
  fast.setAttribute("aria-pressed", ax.fast ? "true" : "false");
  fast.textContent = ax.fast ? `Fast mode ON${mon && mon.poll_rate_hz ? ` · ${Math.round(mon.poll_rate_hz)}/s` : ""}` : "Fast mode";
  renderFault(ax);
  renderRecovery(ax);
  renderControlMode(ax);
}

// ---------------------------------------------------------------- PC / PIO control mode (PMSS)

const MODE_TEXT = {
  pio: "In PIO mode the unit follows its PIO inputs (the PLC) and runs its stored position rows at their own stored " +
    "speeds and accelerations, not the limits set on this page. Motion commands from this page are refused until you " +
    "switch back to PC.\n\nA Modbus STOP is untested on a move a PLC commanded over PIO: stop those from the PLC or " +
    "with the hardware stop.\n\nThe mode is switched only at rest.",
  pc: "The unit takes its commands from this page over Modbus. A PLC that drives it over PIO no longer controls it " +
    "until you switch back to PIO (or disconnect, if PIO had control when it was connected).\n\nThe mode is switched " +
    "only at rest.",
};

function renderControlMode(ax) {
  const st = ax.status || {};
  const pc = st.modbus_enabled;
  const known = pc === true || pc === false;
  const still = st.moving === false && !st.homing && !st.pushing;
  const rest = still && !st.paused;
  // taking PC control from PIO: a pause at a standstill with the servo off does not block it (a PCON in PIO mode with
  // nothing on its PIO connector pauses at rest); a pause with the servo on is a hold; the driver refuses it while
  // anything moves
  const pcOk = pc === false ? still && (!st.paused || st.servo_on === false) : rest;
  $("modePc").setAttribute("aria-pressed", known && pc ? "true" : "false");
  $("modePio").setAttribute("aria-pressed", known && !pc ? "true" : "false");
  $("modePc").disabled = !pcOk || !!ax.fault;
  $("modePio").disabled = !rest || !!ax.fault;
  let text = "";
  if (known && !pc) {
    text = ax.pio_hold
      ? "PIO mode (PMSS = 0, chosen here): the unit follows its PIO inputs and runs its stored rows at their own speeds. " +
        "Motion commands from this page are refused until you switch to PC."
      : "PIO has control (PMSS = 0, as after power-up). The first command takes PC control at rest, or switch here.";
  } else if (!known) {
    text = "The unit does not report who has control (PMSS).";
  }
  if (!(pc === false ? pcOk : rest) && known) text += (text ? " " : "") + "The mode is switched only at rest.";
  const p = $("axControl");
  if (p.textContent !== text) p.textContent = text;
  p.classList.toggle("hidden", !text);
}

async function chooseMode(mode) {
  const ax = selAxis();
  if (!ax) return;
  const pc = (ax.status || {}).modbus_enabled;
  if (pc === (mode === "pc")) return; // already in that mode
  const ok = await ask({
    title: mode === "pio" ? `Switch ${ax.name} to PIO control?` : `Switch ${ax.name} to PC control?`,
    text: MODE_TEXT[mode],
    okText: mode === "pio" ? "Switch to PIO" : "Switch to PC",
  });
  if (!ok) return;
  try {
    await api("/api/wb/call", { axis: ax.name, fn: mode === "pc" ? "enable_modbus_control" : "disable_modbus_control", args: {} });
    toast(mode === "pc" ? `${ax.name}: PC control` : `${ax.name}: PIO control (the unit follows its PIO inputs)`, true);
  } catch (e) {
    toast(`${ax.name}: control mode: ${e.message}`);
  } finally {
    poll();
  }
}
$("modePc").addEventListener("click", () => chooseMode("pc"));
$("modePio").addEventListener("click", () => chooseMode("pio"));

// ---------------------------------------------------------------- power / link loss

const FAULT_TITLES = {
  port_vanished: "Port vanished", unit_silent: "Unit not answering", unit_restarted: "Unit restarted",
  unit_replaced: "Another unit answers",
};

function renderFault(ax) {
  const f = ax.fault;
  $("axFault").classList.toggle("hidden", !f);
  if (!f) return;
  $("axFaultTitle").textContent = `${FAULT_TITLES[f.kind] || "Link fault"}:`;
  $("axFaultText").textContent = f.text || "";
  $("axFaultMsg").textContent = f.message || "";
  const busy = recovering === ax.name;
  $("axRecover").disabled = busy;
  $("axRecover").textContent = busy ? "Recovering…" : "Recover";
}

function renderRecovery(ax) {
  const box = $("axRecovery");
  const r = ax.recovery;
  const sig = r ? `${ax.name}|${r.t}` : "";
  if (box.dataset.sig === sig) return;
  box.dataset.sig = sig;
  box.replaceChildren();
  box.classList.toggle("hidden", !r);
  if (!r) return;
  const rep = r.report || {};
  const u = unitsOf(ax);
  box.classList.toggle("failed", !r.ok);
  box.classList.toggle("attention", r.ok && !!rep.needs_attention);
  box.append(h("div", { class: "rr-head" },
    h("b", { text: r.ok ? "Recovery report" : "Recovery failed" }),
    h("span", { class: "small", text: new Date(r.t * 1000).toLocaleTimeString() }),
    r.ok ? h("span", { class: `tag ${rep.needs_attention ? "unverified" : ""}`, text: rep.needs_attention ? "needs attention" : "ready" }) : null,
    h("span", { class: "small", text: "nothing was moved" }),
    h("button", { type: "button", class: "mini", text: "Dismiss", onclick: () => box.classList.add("hidden") })));
  if (!r.ok) box.append(h("div", { class: "mono", text: r.error || "" }));
  if (rep.summary) box.append(h("div", { text: rep.summary }));
  const facts = [];
  if (rep.homed !== undefined && rep.homed !== null) facts.push(rep.homed ? "Homed" : "NOT homed: home before the next move");
  if (rep.servo_on !== undefined && rep.servo_on !== null) facts.push(`Servo ${rep.servo_on ? "ON" : "OFF"}`);
  if (rep.modbus_control !== undefined && rep.modbus_control !== null) facts.push(`PC control ${rep.modbus_control ? "yes" : "no"}${rep.control_restored ? " (taken again)" : rep.control_kept ? " (kept)" : ""}`);
  if (rep.position_mm !== undefined && rep.position_mm !== null) {
    let p = `Position ${fmtN(rep.position_mm)} ${u}`;
    if (rep.last_target_mm !== null && rep.last_target_mm !== undefined) p += ` (target ${fmtN(rep.last_target_mm)}, ${rep.at_target ? "reached" : rep.status && rep.status.moving ? "still moving: the move sent before the fault runs on" : "NOT reached: the move is not resumed"})`;
    facts.push(p);
  }
  if (rep.alarm_code) facts.push(`Alarm 0x${rep.alarm_code.toString(16).toUpperCase().padStart(3, "0")} ${rep.alarm_name || ""}`);
  if (rep.baudrate) facts.push(`${rep.baudrate} baud${rep.baud_restored ? " (restored)" : ""}`);
  if (facts.length) box.append(h("div", { class: "small", text: facts.join(" · ") }));
  const list = (title, items) => (items && items.length ? h("div", {}, h("b", { class: "small", text: title }), h("ul", {}, items.map((t) => h("li", { text: t })))) : null);
  box.append(...[list("Restart signs", rep.restart_signs), list("Done", rep.actions),
    list("Differences", [...(rep.limits_changed || []), ...(rep.row_problems || []), ...(rep.parameter_differences || [])]),
    list("Notes", rep.notes)].filter(Boolean));
}

async function recoverAxis() {
  const ax = selAxis();
  if (!ax || recovering) return;
  recovering = ax.name;
  renderFault(ax);
  try {
    const v = await api("/api/wb/call", { axis: ax.name, fn: "recover", args: {} });
    const rep = v.result || {};
    toast(rep.summary || "Recovered", !rep.needs_attention);
  } catch (e) {
    toast(e.message);
  } finally {
    recovering = null;
    poll();
  }
}
$("axRecover").addEventListener("click", recoverAxis);

async function toggleFast() {
  const ax = selAxis();
  if (!ax) return;
  try {
    await api("/api/wb/call", { axis: ax.name, fn: "set_fast", args: { on: !ax.fast } });
    toast(ax.fast ? `${ax.name}: fast mode off` : `${ax.name}: fast mode on (live status from the monitor)`, true);
    poll();
  } catch (e) {
    toast(e.message);
  }
}
$("fastToggle").addEventListener("click", toggleFast);

const limitsSig = (ax) => (ax ? ax.name + JSON.stringify(ax.limits) : "");

function maybeBuildForms() {
  const ax = selAxis();
  if (!CATALOG) return;
  const sig = limitsSig(ax);
  if (renderedFor.op !== sig) {
    renderedFor.op = sig;
    buildOperate(ax);
  }
  if (renderedFor.fn !== sig) {
    renderedFor.fn = sig;
    buildFunctions(ax);
  }
  const gsig = STATE.axes.map(limitsSig).join("|");
  if (renderedFor.group !== gsig) {
    renderedFor.group = gsig;
    buildGroup();
  }
}

// ---------------------------------------------------------------- Operate pane

const specOf = (name) => CATALOG.axis.find((f) => f.name === name);

function buildOperate(ax) {
  const box = $("opForms");
  box.replaceChildren();
  if (!ax) return;
  const opts = { axis: ax.name, showResult: false, operating: true };
  const move = fnCard(specOf("move_to"), opts);
  move.querySelector(".fn-actions button").textContent = "Move";
  const rel = fnCard(specOf("move_by"), opts);
  rel.querySelector(".fn-actions button").textContent = "Move by";
  // Jog: a positive step with - / + buttons and preset steps.
  const jogSpec = { ...specOf("jog"), inputs: specOf("jog").inputs.map((i) => (i.name === "delta_mm" ? { ...i, label: "Step", range: "jog_step" } : i)) };
  let jogCard;
  const presets = h("div", { class: "jog-row" },
    h("button", { type: "button", class: "jog-btn", text: "◀ − step", onclick: () => jog(-1) }),
    h("button", { type: "button", class: "jog-btn", text: "+ step ▶", onclick: () => jog(1) }),
    ...[0.1, 1, 10].map((s) => h("button", { type: "button", text: `${s}`, title: `Step ${s} ${unitsOf(ax)}`,
      onclick: () => jogCard.controls.delta_mm.set(s) })));
  jogCard = fnCard(jogSpec, { ...opts, extra: presets });
  jogCard.querySelector(".fn-actions button").classList.add("hidden");
  function jog(sign) {
    let step;
    try {
      step = jogCard.controls.delta_mm.get();
    } catch (e) {
      toast(e.message);
      return;
    }
    jogCard.exec({ delta_mm: sign * Math.abs(step) });
  }
  const push = fnCard(specOf("push"), opts);
  push.querySelector(".fn-actions button").textContent = "Push";
  if (ax.limits && ax.limits.push && ax.limits.push.available === false) {
    push.querySelector(".fn-actions button").disabled = true;
    push.querySelector(".fn-desc").textContent = "This unit reports that it cannot push (SUPT PUSH / MXPP / TCPC).";
  }
  const stored = fnCard(specOf("move_to_position"), opts);
  stored.querySelector(".fn-actions button").textContent = "Run row";
  const servoSpeed = fnCard(specOf("set_safety_speed"), opts);
  box.append(move, jogCard, rel, push, stored, servoSpeed);
}

async function quick(action) {
  const ax = selAxis();
  if (!ax) return;
  const map = {
    servo_on: ["servo", { on: true }], servo_off: ["servo", { on: false }], home: ["home", {}],
    reset_alarm: ["reset_alarm", {}], pause: ["pause", {}], resume: ["resume", {}], stop: ["stop", {}],
    cancel_pause: ["cancel_paused_move", {}],
  };
  const [fn, args] = map[action];
  try {
    await api("/api/wb/call", { axis: ax.name, fn, args });
    poll();
  } catch (e) {
    toast(e.message);
  }
}
for (const b of document.querySelectorAll("[data-quick]")) b.addEventListener("click", () => quick(b.dataset.quick));

// ---------------------------------------------------------------- views and sub tabs

function currentPane() {
  const b = document.querySelector("#axisTabs button[data-pane].active");
  return b ? b.dataset.pane : "paneOperate";
}
for (const b of document.querySelectorAll("#axisTabs button[data-pane]")) {
  b.addEventListener("click", () => {
    for (const o of document.querySelectorAll("#axisTabs button[data-pane]")) o.classList.toggle("active", o === b);
    renderHead();
    if (b.dataset.pane === "panePositions") loadPositions();
    if (b.dataset.pane === "paneParams") loadParams();
    if (b.dataset.pane === "paneInfo") loadOverview();
  });
}

function currentView() {
  const b = document.querySelector(".views button[data-view].active");
  return b ? b.dataset.view : "viewAxis";
}

function showView(id) {
  if (!VIEWS.includes(id) || !$(id)) return;
  for (const o of document.querySelectorAll(".views button[data-view]")) {
    const on = o.dataset.view === id;
    o.classList.toggle("active", on);
    o.setAttribute("aria-selected", on ? "true" : "false");
    $(o.dataset.view).classList.toggle("hidden", !on);
  }
  store(KEY_VIEW, id);
  updateEmpties();
  for (const fn of viewListeners) {
    try {
      fn(id);
    } catch (e) {
      noteError(`Page error: ${e.message}`);
    }
  }
}
for (const b of document.querySelectorAll(".views button[data-view]")) b.addEventListener("click", () => showView(b.dataset.view));

// The last view comes back after a reload (once every script has registered its listeners).
document.addEventListener("DOMContentLoaded", () => {
  const v = stored(KEY_VIEW);
  if (v && VIEWS.includes(v) && v !== currentView()) showView(v);
});

// ---------------------------------------------------------------- API toggle

function setShowApi(on, save = true) {
  document.body.classList.toggle("show-api", on);
  for (const b of document.querySelectorAll("#apiToggle, [data-api-toggle]")) b.setAttribute("aria-pressed", on ? "true" : "false");
  if (save) store(KEY_API, on ? "1" : "0");
}
for (const b of document.querySelectorAll("#apiToggle, [data-api-toggle]")) {
  b.addEventListener("click", () => setShowApi(!document.body.classList.contains("show-api")));
}
setShowApi(stored(KEY_API) === "1", false);

// ---------------------------------------------------------------- stored positions

const POS_COLS = [
  ["position_mm", "Position", "position"], ["speed_mm_s", "Speed", "speed"], ["accel_g", "Accel", "accel"],
  ["decel_g", "Decel", "decel"], ["band_mm", "Band", "push_band"], ["push_pct", "Push %", "push0"],
  ["zone_plus_mm", "Zone +", null], ["zone_minus_mm", "Zone −", null],
];

async function loadPositions() {
  const ax = selAxis();
  if (!ax) return;
  const table = $("posTable");
  table.replaceChildren(h("tr", {}, h("td", { class: "small", text: "Reading rows from the unit..." })));
  let v;
  try {
    v = await api(`/api/wb/positions?axis=${encodeURIComponent(ax.name)}`);
  } catch (e) {
    table.replaceChildren(h("tr", {}, h("td", { class: "error", text: e.message })));
    return;
  }
  const lim = v.limits || {};
  $("posInfo").textContent = `${v.rows.length} rows · units ${lim.units || "mm"}`;
  if (!v.rows.length) {
    table.replaceChildren(h("tr", {}, h("td", { class: "small", text: "The unit reports no stored rows." })));
    return;
  }
  const runPct = buildInput({ name: "speed_pct", label: "Run speed %", type: "number", range: "speed_override", example: 100, optional: true }, lim);
  const head = h("tr", {}, h("th", { text: "Row" }),
    POS_COLS.map(([, label, range]) => {
      const r = range ? rangeFor({ range }, lim) : null;
      return h("th", {}, label, h("span", { class: "range", text: r ? rangeText(r, {}) : lim.units || "" }));
    }),
    h("th", { text: "Mode" }), h("th", { text: "Flags" }), h("th", {}, "Actions", runPct.el));
  table.replaceChildren(head);
  for (const row of v.rows) {
    if (row.error) {
      table.append(h("tr", {}, h("td", { text: row.n }), h("td", { colspan: 12, class: "error", text: row.error })));
      continue;
    }
    const inputs = {};
    const tr = h("tr", {});
    tr.append(h("td", { class: "mono", text: row.n }));
    for (const [key, label, range] of POS_COLS) {
      const spec = { name: key, label: `Row ${row.n} ${label}`, type: "number", range: range || undefined, optional: key.startsWith("zone") };
      if (!range) spec.min = -99999, spec.max = 99999; // zones: any value the unit accepts
      const c = buildInput(spec, lim, { prefill: false, compact: true });
      c.set(row[key]);
      c.field.addEventListener("input", () => tr.classList.add("dirty"));
      inputs[key] = c;
      tr.append(h("td", {}, c.el));
    }
    const mode = h("select", {}, [["normal", "normal"], ["push+", "push +"], ["push-", "push −"]].map(([val, t]) => h("option", { value: val, text: t })));
    mode.value = row.mode;
    mode.addEventListener("change", () => tr.classList.add("dirty"));
    tr.append(h("td", {}, mode), h("td", { class: "mono small", text: row.control_flags + (row.checksum_ok === false ? " ✗sum" : "") }));
    const runBtn = h("button", { type: "button", class: "mini", text: "Run", title: `Move ${ax.name} to stored row ${row.n}`, onclick: async () => {
      let pct;
      try { pct = runPct.get(); } catch (e) { toast(e.message); return; }
      try {
        await api("/api/wb/call", { axis: ax.name, fn: "move_to_position", args: { n: row.n, ...(pct !== undefined ? { speed_pct: pct } : {}) } });
        poll();
      } catch (e) { toast(e.message); }
    } });
    const writeBtn = h("button", { type: "button", class: "mini", text: "Write…", title: `Write row ${row.n} to the unit's non-volatile memory`, onclick: async () => {
      const args = { n: row.n, mode: mode.value };
      try {
        for (const [k, c] of Object.entries(inputs)) {
          const val = c.get();
          if (val !== undefined) args[k] = val;
        }
      } catch (e) { toast(e.message); return; }
      const ok = await ask({
        title: `Write row ${row.n} of ${ax.name}?`,
        text: `This changes stored position ${row.n} in the unit's NON-VOLATILE memory (PERMANENT until changed again):\n` +
          POS_COLS.map(([k, l]) => `  ${l}: ${row[k]} → ${args[k] ?? row[k]}`).join("\n") + `\n  Mode: ${row.mode} → ${args.mode}\n\nBack up the unit first.` +
          nvCountLine(ax.name),
        okText: "Write row",
      });
      if (!ok) return;
      try {
        const v = await api("/api/wb/call", { axis: ax.name, fn: "write_position_table", args });
        toast(`Row ${row.n}: ${nvResultText(v.result)}`, true);
        loadPositions();
        poll();
      } catch (e) { toast(e.message); }
    } });
    tr.append(h("td", { class: "actions" }, runBtn, writeBtn));
    table.append(tr);
  }
}
$("posReload").addEventListener("click", loadPositions);

// ---------------------------------------------------------------- parameters

async function loadParams() {
  const ax = selAxis();
  if (!ax) return;
  $("paramTable").replaceChildren(h("tr", {}, h("td", { class: "small", text: "Reading parameters from the unit..." })));
  try {
    const v = await api(`/api/wb/parameters?axis=${encodeURIComponent(ax.name)}`);
    paramsCache = v.parameters;
  } catch (e) {
    $("paramTable").replaceChildren(h("tr", {}, h("td", { class: "error", text: e.message })));
    return;
  }
  renderParams();
}

function choiceList(choices) {
  if (!choices) return [];
  if (Array.isArray(choices)) return choices.map((c) => (Array.isArray(c) ? c : [c, c]));
  return Object.entries(choices);
}

function renderParams() {
  const ax = selAxis();
  const filter = $("paramFilter").value.trim().toLowerCase();
  const table = $("paramTable");
  table.replaceChildren(h("tr", {}, ["No.", "Name", "Value", "Unit", "Range", "Item", ""].map((t) => h("th", { text: t }))));
  let shown = 0;
  for (const p of paramsCache) {
    const label = `${p.number} ${p.name}`.toLowerCase();
    if (filter && !label.includes(filter)) continue;
    shown += 1;
    const choices = choiceList(p.choices);
    let control;
    if (choices.length) {
      control = h("select", {}, choices.map(([k, t]) => h("option", { value: k, text: String(t).startsWith(`${k}:`) ? t : `${k}: ${t}` })));
      const cur = choices.find(([k, t]) => String(k) === String(p.value) || String(p.text || "").startsWith(`${k}:`) || t === p.text);
      if (cur) control.value = cur[0];
    } else {
      control = h("input", { type: "text", class: "wide-in", value: p.value === null || p.value === undefined ? "" : String(p.value),
        placeholder: p.min !== null && p.min !== undefined ? `e.g. ${fmtN(p.min)}` : "" });
    }
    if (!p.writable) control.disabled = true;
    const range = p.min !== null && p.min !== undefined ? `${fmtN(p.min)} – ${fmtN(p.max)}` : choices.length ? "choice" : "";
    const tr = h("tr", { class: p.dangerous ? "dangerous" : "" },
      h("td", { class: "mono", text: p.number }),
      h("td", {}, p.name, " ", p.dangerous ? h("span", { class: "tag danger", text: "dangerous" }) : null),
      h("td", {}, control),
      h("td", { text: p.unit || "" }),
      h("td", { class: "small", text: range }),
      h("td", { class: "mono small", text: `${p.item || ""} ${p.raw ? "· " + p.raw : ""}` }),
      h("td", {}, p.writable ? h("button", { type: "button", class: "mini", text: "Write…", onclick: () => writeParam(ax, p, control) })
        : h("span", { class: "small", text: "read-only" })));
    table.append(tr);
  }
  if (!shown) {
    table.append(h("tr", {}, h("td", { colspan: 7, class: "small",
      text: paramsCache.length ? "No parameter matches the filter." : "The unit lists no parameters." })));
  }
}
$("paramFilter").addEventListener("input", renderParams);
$("paramReload").addEventListener("click", loadParams);

async function writeParam(ax, p, control) {
  const raw = control.value.trim();
  if (raw === "") { toast(`Enter a value for parameter ${p.number}.`); return; }
  if (!control.tagName.match(/select/i) && p.min !== null && p.min !== undefined) {
    const v = parseNumber(raw, false);
    if (Number.isNaN(v) || v < p.min - 1e-9 || v > p.max + 1e-9) {
      toast(`Parameter ${p.number} (${p.name}) must be ${fmtN(p.min)} – ${fmtN(p.max)} ${p.unit || ""}.`);
      return;
    }
  }
  const ok = await ask({
    title: `Write parameter ${p.number} on ${ax.name}?`,
    text: `${p.name}: ${p.text ?? p.value} → ${raw} ${p.unit || ""}\n\nPERMANENT: verified by read-back and saved to non-volatile memory; ` +
      `it applies after a SOFTWARE RESET (Unit info tab) or a power cycle.` +
      (p.dangerous ? "\n\nDANGEROUS parameter (home direction / offset, encoder / motor type, lead / stroke, gains, " +
        "axis number or baud): a wrong value can make the axis move unexpectedly. Back up the unit first." : "") +
      nvCountLine(ax.name),
    okText: "Write", danger: !!p.dangerous,
  });
  if (!ok) return;
  try {
    const args = { key: String(p.number), value: raw };
    const v = await api("/api/wb/call", { axis: ax.name, fn: "write_parameter", args });
    const r = v.result || {};
    const effect = !r.skipped && r.takes_effect ? "; " + r.takes_effect : "";
    const warn = (r.warnings || []).length ? `. Warning: ${r.warnings.join("; ")}` : "";
    toast(`Parameter ${p.number}: ${nvResultText(r)}${effect}${warn}`, true);
    loadParams();
    poll();
  } catch (e) {
    toast(e.message);
  }
}

// ---------------------------------------------------------------- unit info

function kvRows(table, rows) {
  table.replaceChildren(...rows.filter(([, v]) => v !== undefined).map(([k, v]) =>
    h("tr", {}, h("th", { text: k }), h("td", { text: v === null || v === "" ? "–" : String(v) }))));
}

async function loadOverview() {
  const ax = selAxis();
  if (!ax) return;
  let v;
  try {
    v = await api(`/api/wb/overview?axis=${encodeURIComponent(ax.name)}`);
  } catch (e) {
    toast(e.message);
    return;
  }
  const i = v.info || {};
  const l = v.limits || {};
  const u = l.units || "mm";
  kvRows($("identityTable"), [
    ["Axis", `${v.axis} (axis no. ${ax.axis_number}, slave ${ax.slave})`], ["Model", i.model], ["Controller", i.controller],
    ["Actuator", i.actuator], ["Kind", l.kind || i.kind], ["Family", l.family || i.product_family], ["Units", u],
    ["Firmware", i.firmware], ["Serial", i.serial_number], ["System / unit", [i.system_name, i.unit_name].filter(Boolean).join(" / ")],
    ["Stroke", l.position ? `${fmtN(l.position.min)} – ${fmtN(l.position.max)} ${u}` : null],
    ["Max speed", l.speed ? `${fmtN(l.speed.max)} ${u}/s` : null],
    ["Max accel / decel", l.accel ? `${fmtN(l.accel.max)} / ${fmtN(l.decel.max)} G` : null],
    ["Push", l.push ? (l.push.available === false ? "not available" : `${fmtN(l.push.min)} – ${fmtN(l.push.max)} % (recommended ≥ ${fmtN(l.push.recommended_min)} %)`) : null],
    ["Stored rows", l.stored_rows ? l.stored_rows.count : null],
    ["Port", v.port ? `${v.port.name} · ${v.port.baud || "no"} baud · ${v.port.axes.length} axis on this port${v.port.line && v.port.line !== "direct" ? " · " + v.port.line + " line (see Functions: Line and gateway)" : ""}` : null],
  ]);
  const m = v.monitor || {};
  kvRows($("monitorTable"), v.monitor ? [
    ["Position", `${fmtN(m.position)} ${u}`], ["Velocity", `${fmtN(m.velocity)} ${u}/s`], ["Current", m.current_pct !== null && m.current_pct !== undefined ? `${fmtN(m.current_pct, 1)} % (raw ${m.current})` : m.current],
    ["Deviation", m.deviation], ["Alarm", m.alarm ? `${Number(m.alarm).toString(16).toUpperCase()} ${m.alarm_name || ""}` : "none"],
    ["Uptime", m.uptime_s !== null && m.uptime_s !== undefined ? `${fmtN(m.uptime_s, 1)} s` : null],
  ] : [["Monitor", v.monitor_error || "not readable"]]);
  const box = $("alarmNow");
  if (v.alarm) {
    box.replaceChildren(h("div", { class: `alarm-box ${v.alarm.warning ? "warn" : ""}` },
      h("b", { text: `${v.alarm.hex} ${v.alarm.name}` }), ` · ${v.alarm.level_text || v.alarm.level}`,
      h("div", { text: `Remedy: ${v.alarm.remedy}` }), v.alarm.reset ? h("div", { text: `Reset: ${v.alarm.reset}` }) : null));
  } else box.textContent = "No alarm.";
  const ht = $("historyTable");
  ht.replaceChildren(h("tr", {}, ["#", "Code", "Name", "Level", "Time after power-on", "Remedy"].map((t) => h("th", { text: t }))));
  for (const r of v.history || []) {
    ht.append(h("tr", {}, h("td", { text: r.index }), h("td", { class: "mono", text: Number(r.code).toString(16).toUpperCase().padStart(3, "0") }),
      h("td", { text: r.name }), h("td", { text: r.level }), h("td", { class: "mono small", text: r.time_after_power_on || "–" }),
      h("td", { class: "small", text: r.power_up ? "" : r.remedy })));
  }
  if (!(v.history || []).length && !v.history_error) ht.append(h("tr", {}, h("td", { colspan: 6, class: "small", text: "No alarm in the history." })));
  if (v.history_error) ht.append(h("tr", {}, h("td", { colspan: 6, class: "error", text: v.history_error })));
}
$("infoReload").addEventListener("click", loadOverview);

// Self-test (iai-check, read-only): the full result from this page, else the last summary the server keeps.
const RESULT_ORDER = ["PASS", "WARN", "FAIL", "SKIP"];
function countsText(c) {
  if (!c) return "";
  const keys = [...RESULT_ORDER.filter((k) => k in c), ...Object.keys(c).filter((k) => !RESULT_ORDER.includes(k))];
  return keys.map((k) => `${c[k]} ${k}`).join(", ");
}
function resultChip(res) {
  const r = String(res || "?").toUpperCase();
  return h("span", { class: `tag res res-${r.toLowerCase()}`, text: r });
}
function renderSelfCheck(ax) {
  const card = $("selfCheckCard");
  const table = $("selfCheckTable");
  const full = ax ? selfChecks[ax.name] : null;
  const last = full || (ax && ax.check) || null;
  card.classList.toggle("hidden", !last);
  const sig = last ? `${ax.name}|${last.t}|${full ? "items" : "summary"}` : "";
  if (table.dataset.sig === sig) return;
  table.dataset.sig = sig;
  if (!last) {
    table.replaceChildren();
    return;
  }
  const when = last.t ? ` at ${new Date(last.t * 1000).toLocaleTimeString()}` : "";
  const counts = countsText(last.counts);
  const caption = h("caption", {}, resultChip(last.result), `${when}${counts ? ` (${counts})` : ""}`,
    full ? "" : ". Run it again to see each check.");
  const items = full ? full.items || [] : [];
  table.replaceChildren(caption, ...(items.length ? [
    h("tr", {}, ["Check", "Result", "Detail"].map((t) => h("th", { text: t }))),
    ...items.map((it) => h("tr", { class: `res-row res-row-${String(it.result || "").toLowerCase()}` },
      h("td", { class: "mono", text: it.name }), h("td", {}, resultChip(it.result)), h("td", { class: "small", text: it.detail || "" }))),
  ] : []));
}

async function runSelfCheck() {
  const ax = selAxis();
  if (!ax) return;
  const btn = $("selfCheckBtn");
  btn.disabled = true;
  btn.textContent = "Self-test running...";
  try {
    const v = await api("/api/wb/call", { axis: ax.name, fn: "self_check", args: {} });
    const r = v.result || v;
    selfChecks[ax.name] = r;
    renderSelfCheck(ax);
    toast(`${ax.name}: self-test ${r.result || "done"}${r.counts ? ` (${countsText(r.counts)})` : ""}`, r.passed !== false);
    poll();
  } catch (e) {
    toast(`${ax.name}: self-test failed: ${e.message}`);
  } finally {
    btn.disabled = false;
    btn.textContent = "Self-test (read-only)";
  }
}
$("selfCheckBtn").addEventListener("click", runSelfCheck);

$("baudForm").addEventListener("submit", async (ev) => {
  ev.preventDefault();
  const ax = selAxis();
  if (!ax) return;
  const rate = Number($("baudRate").value);
  const spec = CATALOG.port.find((f) => f.name === "set_baudrate");
  const verified = (spec.inputs[0].choices.find(([r]) => r === rate) || [0, ""])[1].indexOf("UNVERIFIED") < 0;
  const ok = await ask({
    title: `Change ${ax.port} to ${rate} baud?`,
    text: `Broadcast baud change on ${ax.port}: EVERY unit on this port switches to ${rate} baud (volatile: back to the power-up rate after a power cycle).` +
      (verified ? "" : `\n\n${rate} is an UNVERIFIED rate code: the unit may stop answering until a power cycle.`),
    okText: "Change baud", danger: !verified,
  });
  if (!ok) return;
  try {
    await api("/api/wb/port", { port: ax.port, fn: "set_baudrate", args: { rate, method: $("baudMethod").value } });
    toast(`${ax.port} now at ${rate} baud`, true);
    poll();
  } catch (e) {
    toast(e.message);
  }
});

$("swReset").addEventListener("click", async () => {
  const ax = selAxis();
  if (!ax) return;
  if (!(await confirmFor(specOf("software_reset"), {}, ax.name))) return;
  try {
    await api("/api/wb/call", { axis: ax.name, fn: "software_reset", args: {} });
    toast(`${ax.name}: software reset done`, true);
    loadOverview();
  } catch (e) {
    toast(e.message);
  }
});

// ---------------------------------------------------------------- functions view

function buildFunctions(ax) {
  const list = $("fnList");
  list.replaceChildren();
  if (!ax) {
    list.append(h("p", { class: "small", text: "Connect an axis (or add a simulator) to run functions." }));
    return;
  }
  const groups = {};
  for (const f of CATALOG.axis) (groups[f.group] = groups[f.group] || []).push(fnCard(f, { axis: ax.name }));
  groups.Port = CATALOG.port.map((f) => fnCard(f, { axis: ax.name, portFn: true }));
  for (const [g, cards] of Object.entries(groups)) {
    list.append(h("section", { class: "fn-group", dataset: { group: g } }, h("h2", { text: g === "Port" ? `Port (${ax.port})` : g }), h("div", { class: "op-grid" }, cards)));
  }
  filterFunctions();
}
function filterFunctions() {
  const q = $("fnFilter").value.trim().toLowerCase();
  for (const c of document.querySelectorAll("#fnList .fn-card")) c.classList.toggle("hidden", !!q && !c.textContent.toLowerCase().includes(q));
}
$("fnFilter").addEventListener("input", filterFunctions);

// ---------------------------------------------------------------- group view

const groupInputs = {};

function buildGroup() {
  const table = $("groupTable");
  table.replaceChildren(h("tr", {}, ["", "Axis", "Model / kind", "Position", "Status", "Target", "Speed"].map((t) => h("th", { text: t }))));
  for (const ax of STATE.axes) {
    const lim = ax.limits || {};
    const prev = groupInputs[ax.name] || {};
    const target = buildInput({ name: "target", label: `${ax.name} target`, type: "number", range: "position", example: "frac:0.5" }, lim, { prefill: false, compact: true });
    const speed = buildInput({ name: "speed", label: `${ax.name} speed`, type: "number", range: "speed", example: "cap:50" }, lim, { compact: true });
    const sel = h("input", { type: "checkbox", "aria-label": `Select ${ax.name}` });
    sel.checked = prev.sel ? prev.sel.checked : true;
    if (prev.target) target.set(prev.target.field.value);
    if (prev.speed) speed.set(prev.speed.field.value);
    groupInputs[ax.name] = { sel, target, speed };
    table.append(h("tr", { dataset: { axis: ax.name } }, h("td", {}, sel), h("td", {}, h("b", { text: ax.name })),
      h("td", { class: "small", text: `${(ax.info && ax.info.model) || "?"} · ${lim.kind || "?"}` }),
      h("td", { class: "mono g-pos" }), h("td", { class: "g-lamps" }), h("td", {}, target.el), h("td", {}, speed.el)));
  }
  renderGroupLive();
}

function renderGroupLive() {
  for (const tr of document.querySelectorAll("#groupTable tr[data-axis]")) {
    const ax = axisOf(tr.dataset.axis);
    if (!ax) continue;
    const st = ax.status || {};
    tr.querySelector(".g-pos").textContent = `${st.position_mm === undefined || st.position_mm === null ? "–" : fmtN(st.position_mm)} ${unitsOf(ax)}`;
    tr.querySelector(".g-lamps").replaceChildren(...[h("div", { class: "mini-lamps" }, lampsFor(ax.status).filter(Boolean).slice(0, 8)),
      ax.busy ? h("div", { class: "small", text: `busy: ${ax.busy}` }) : null].filter(Boolean));
  }
}

const selectedAxes = () => STATE.axes.map((a) => a.name).filter((n) => groupInputs[n] && groupInputs[n].sel.checked);

$("grpSelectAll").addEventListener("click", () => {
  const all = Object.values(groupInputs).every((g) => g.sel.checked);
  for (const g of Object.values(groupInputs)) g.sel.checked = !all;
});
$("grpMove").addEventListener("click", async () => {
  const names = selectedAxes();
  if (!names.length) { toast("Select at least one axis."); return; }
  const targets = {}, speed = {};
  try {
    for (const n of names) {
      targets[n] = groupInputs[n].target.get();
      speed[n] = groupInputs[n].speed.get();
    }
  } catch (e) { toast(e.message); return; }
  try {
    await api("/api/wb/group", { op: "move_all", axes: names, targets, speed });
    poll();
  } catch (e) { toast(e.message); }
});
async function groupOp(op, extra = {}) {
  const names = selectedAxes();
  if (!names.length) { toast("Select at least one axis."); return; }
  try {
    const v = await api("/api/wb/group", { op, axes: names, ...extra });
    const errs = Object.entries(v.errors || {});
    if (errs.length) toast(errs.map(([n, e]) => `${n}: ${e}`).join("\n"));
    poll();
  } catch (e) { toast(e.message); }
}
$("grpHome").addEventListener("click", () => groupOp("home_all"));
$("grpServoOn").addEventListener("click", () => groupOp("servo_all", { on: true }));
$("grpServoOff").addEventListener("click", () => groupOp("servo_all", { on: false }));
$("grpStop").addEventListener("click", stopAll);

// ---------------------------------------------------------------- STOP

// STOP goes round the request budget, is cut off after STOP_TIMEOUT_MS and tried once more. A STOP that is still not
// answered (or was refused) leaves a red banner until a later STOP ALL is answered: the operator must not assume
// that the axes have stopped.
const STOP_TIMEOUT_MS = 2000;
function stopBanner(failure) {
  const banner = $("stopBanner");
  banner.classList.toggle("hidden", !failure);
  banner.textContent = failure ? `STOP NOT CONFIRMED (${failure}): use the emergency stop. This stays until a STOP ALL is answered.` : "";
}
async function stopRequest(data) {
  let failure = "";
  for (let attempt = 0; attempt < 2; attempt++) {
    const abort = new AbortController();
    const timer = setTimeout(() => abort.abort(), STOP_TIMEOUT_MS);
    try {
      return await rawApi("/api/wb/stop", data, abort.signal);
    } catch (e) {
      failure = e.message;
    } finally {
      clearTimeout(timer);
    }
  }
  throw httpError(failure, 0, null);
}
async function stopAll() {
  const b = $("stopAll");
  b.classList.add("pressed");
  setTimeout(() => b.classList.remove("pressed"), 300);
  try {
    const broadcast = $("grpBroadcast") && $("grpBroadcast").checked;
    const v = await stopRequest(broadcast ? { broadcast: true } : {});
    const errs = Object.entries(v.errors || {});
    if (errs.length) {
      const text = errs.map(([n, e]) => `${n}: ${e}`).join("\n");
      stopBanner(`failed on ${text}`);
      toast(`STOP failed on ${text}`);
    } else {
      stopBanner("");
    }
  } catch (e) {
    stopBanner(e.message);
    toast(`STOP ALL failed: ${e.message}`);
  }
  poll();
}
async function stopAxis(name) {
  try {
    await stopRequest({ axis: name });
  } catch (e) {
    toast(`STOP ${name} failed: ${e.message}`);
  }
  poll();
}
$("stopAll").addEventListener("click", stopAll);
// The only keydown handler of the page: Esc always stops everything, Space unless the focus is in a text field.
document.addEventListener("keydown", (ev) => {
  if (ev.key === "Escape") {
    stopAll(); // always, even with a dialog open (the dialog closes as "Cancel")
    return;
  }
  if ($("confirmDlg").open) return;
  const t = ev.target;
  const typing = t && (t.tagName === "TEXTAREA" || (t.tagName === "INPUT" && !["checkbox", "radio", "button"].includes(t.type)) || t.isContentEditable);
  if (ev.key === " " && !typing) {
    ev.preventDefault();
    stopAll();
  }
});

// ---------------------------------------------------------------- connection

let portsLoaded = false; // the first listing ticks the IAI adapters
async function loadPorts(announce = true) {
  let v;
  try {
    v = await api("/api/wb/ports");
  } catch (e) {
    if (announce) toast(e.message);
    return;
  }
  const ul = $("portList");
  const checked = new Set([...ul.querySelectorAll("input:checked")].map((i) => i.value));
  const first = !portsLoaded;
  portsLoaded = true;
  ul.replaceChildren();
  const item = (p) => {
    const box = h("input", { type: "checkbox", value: p.port, disabled: p.connected });
    box.checked = !p.connected && (checked.has(p.port) || (p.is_iai && (first || announce) && !STATE.ports.length));
    box.addEventListener("change", updateConnectBtn);
    return h("li", {}, h("label", { class: p.connected ? "busy" : "" }, box,
      h("div", { class: "grow" }, h("b", { text: p.port }), " ", p.is_iai ? h("span", { class: "tag", text: "IAI" }) : null,
        p.connected ? h("span", { class: "tag sim", text: "connected" }) : null,
        h("div", { class: "desc", text: p.description || "", title: p.description || "" }))));
  };
  const iai = v.ports.filter((p) => p.is_iai);
  const other = v.ports.filter((p) => !p.is_iai);
  if (!v.ports.length) {
    ul.append(h("li", { class: "small", text: "No serial port found. Plug in the adapter and press Refresh, or use a simulator below." }));
  } else {
    ul.append(h("li", { class: "port-group label", text: "IAI adapters" }));
    if (iai.length) ul.append(...iai.map(item));
    else ul.append(h("li", { class: "small", text: "No IAI adapter found." }));
    if (other.length) ul.append(h("li", { class: "port-group label", text: "Other ports" }), ...other.map(item));
  }
  const sim = $("simSetup");
  if (!sim.options.length) for (const s of v.sim_setups) sim.append(h("option", { value: s.name, text: s.label }));
  applyDemoMode(!!v.sim_only);
  updateConnectBtn();
}

// Demo mode (--sim-only): no serial port is listed or opened, so the serial controls give way to the simulator.
let emptyNodes = null; // the hardware wording of the empty state (its nodes), kept while the demo wording is shown
function applyDemoMode(on) {
  for (const id of ["serialHead", "portList", "serialFields", "connectPorts", "serialNote"]) $(id).classList.toggle("hidden", on);
  if (on) $("connMsg").classList.add("hidden");
  $("startDemoNote").classList.toggle("hidden", !on);
  const p = $("emptyText");
  if (on && emptyNodes === null) {
    emptyNodes = [...p.childNodes];
    p.replaceChildren(
      "Demo mode: press Add two simulated axes, or choose a simulator on the left and press Add simulator, e.g. " +
      "ELECYLINDER slider for one axis, Multi-axis line for three axes on one port, or Cell for three ports. Then try " +
      "Servo ON, Home and the Operate panel. STOP is always in the header (Space or Esc). Every range shown comes " +
      "from the simulated unit.");
  } else if (!on && emptyNodes !== null) {
    p.replaceChildren(...emptyNodes);
    emptyNodes = null;
  }
}
function updateConnectBtn() {
  $("connectPorts").disabled = !$("portList").querySelector("input:checked");
}
$("refreshPorts").addEventListener("click", () => loadPorts(true));

// Connect messages: the server's text as it is, with a short title for the error kind.
const CONN_KINDS = {
  in_use: "Port in use", no_permission: "No permission", not_found: "Port not found",
  no_answer: "Nothing answered", open_failed: "Cannot open the port",
};
function showConnMsg(lines) {
  const p = $("connMsg");
  p.replaceChildren();
  if (!lines || !lines.length) {
    p.className = "conn-msg hidden";
    return;
  }
  p.className = `conn-msg ${lines.some((l) => l.kind !== "ok") ? "bad" : "ok"}`;
  for (const l of lines) {
    p.append(h("span", { class: `cm-line ${l.kind}` }, CONN_KINDS[l.kind] ? h("b", { text: `${CONN_KINDS[l.kind]}: ` }) : null, l.text));
  }
}

$("connectPorts").addEventListener("click", async () => {
  const ports = [...$("portList").querySelectorAll("input:checked")].map((i) => i.value);
  if (!ports.length) return;
  const btn = $("connectPorts");
  btn.disabled = true;
  btn.textContent = `Scanning ${ports.length} port${ports.length === 1 ? "" : "s"}...`;
  showConnMsg(null);
  const baud = $("connBaud").value;
  const auto = baud === "auto";
  const results = await Promise.allSettled(ports.map((port) =>
    api("/api/wb/connect", { port, slaves: $("scanSlaves").value, baud: auto ? null : Number(baud) })));
  const lines = [];
  results.forEach((r, i) => {
    if (r.status === "fulfilled") {
      for (const p of r.value.connected || []) {
        const how = p.baud ? `${p.baud} baud ${auto ? "(detected)" : "(set)"}` : "no baud";
        lines.push({ kind: "ok", text: `${p.name}: connected at ${how}, axes ${(p.axes || []).join(", ")}.` });
      }
    } else {
      const e = r.reason || {};
      const text = String(e.message || e);
      lines.push({ kind: e.kind || "error", text: text.includes(ports[i]) ? text : `${ports[i]}: ${text}` });
    }
  });
  showConnMsg(lines);
  const errs = lines.filter((l) => l.kind !== "ok");
  if (errs.length) toast(errs.map((l) => l.text).join("\n"));
  btn.textContent = "Connect selected ports";
  await poll();
  loadPorts(false);
});

$("addSim").addEventListener("click", async () => {
  const btn = $("addSim");
  btn.disabled = true;
  try {
    await api("/api/wb/connect", { simulate: true, sim_setup: $("simSetup").value });
    await poll();
  } catch (e) {
    toast(e.message);
  } finally {
    btn.disabled = false;
  }
});

async function scanPort(name) {
  try {
    const v = await api("/api/wb/port", { port: name, fn: "scan", args: { slaves: $("scanSlaves").value } });
    toast(v.added.length ? `${name}: added ${v.added.join(", ")}` : `${name}: no new axis`, true);
    poll();
  } catch (e) {
    toast(e.message);
  }
}
async function disconnectPort(name) {
  try {
    const v = await api("/api/wb/disconnect", { port: name });
    const axes = Object.entries(v.axes || {});
    const unconfirmed = axes.filter(([, a]) => a && a.stop_confirmed === false).map(([n]) => n);
    const what = axes.map(([n, a]) => `${n} ${(a && a.note) || "closed"}`).join("; ");
    if (unconfirmed.length) toast(`${name} disconnected, STOP NOT CONFIRMED on ${unconfirmed.join(", ")}: ${what}`);
    else toast(what ? `${name} disconnected: ${what}` : `${name} disconnected`, true);
    for (const k of Object.keys(selfChecks)) if (k.startsWith(`${name}:`)) delete selfChecks[k];
    await poll();
    loadPorts(false);
  } catch (e) {
    toast(e.message);
  }
}

// ---------------------------------------------------------------- simulated pair and Start here

// Two simulated sliders on two ports (setup "xy"): enough for the Group and Coordinated views.
async function addSimPair() {
  const v = await api("/api/wb/connect", { simulate: true, sim_setup: "xy" });
  await poll();
  return v;
}

async function addPairAndSelect(btn) {
  if (btn) btn.disabled = true;
  try {
    const v = await addSimPair();
    const names = (v.connected || []).flatMap((p) => p.axes || []);
    if (names.length && axisOf(names[0])) selectAxis(names[0]);
    toast(names.length ? `Added ${names.join(" and ")} (simulated sliders)` : "Simulated axes added", true);
  } catch (e) {
    toast(`Add two simulated axes: ${e.message}`);
  } finally {
    if (btn) btn.disabled = false;
  }
}
$("startDemo").addEventListener("click", (ev) => addPairAndSelect(ev.currentTarget));
for (const b of document.querySelectorAll("[data-add-pair]")) b.addEventListener("click", () => addPairAndSelect(b));

function showStart(show) {
  $("startHere").classList.toggle("hidden", !show);
  $("startBtn").setAttribute("aria-expanded", show ? "true" : "false");
}
$("startHide").addEventListener("click", () => {
  showStart(false);
  store(KEY_START, "1");
});
$("startBtn").addEventListener("click", () => {
  showStart(true);
  $("startHere").scrollIntoView({ block: "start" });
});
showStart(stored(KEY_START) !== "1");

// ---------------------------------------------------------------- test report

let reportFile = "iai-gui-report.txt";

function screenText() {
  const s = window.screen || {};
  const scale = Math.round((window.devicePixelRatio || 1) * 100) / 100;
  return `${s.width}x${s.height}, scale ${scale}, window ${window.innerWidth}x${window.innerHeight}`;
}

async function fetchReport() {
  const v = await api("/api/wb/report", {
    browser: navigator.userAgent,
    screen: screenText(),
    page: `workbench, ${VIEW_NAMES[currentView()] || currentView()} view`,
    client_errors: clientErrors.slice(-30),
  });
  if (v.filename) reportFile = v.filename;
  return v.text || "";
}

// Clipboard API first; the textarea-select copy where the browser does not allow it.
async function copyText(text) {
  try {
    if (navigator.clipboard && window.isSecureContext) {
      await navigator.clipboard.writeText(text);
      return true;
    }
  } catch {
    // fall back below
  }
  const inDialog = $("reportDlg").open;
  const ta = inDialog ? $("reportText") : h("textarea", { class: "offscreen", readonly: true, "aria-hidden": "true" });
  if (!inDialog) document.body.append(ta);
  ta.value = text;
  ta.focus();
  ta.select();
  let ok = false;
  try {
    ok = document.execCommand("copy");
  } catch {
    ok = false;
  }
  if (!inDialog) ta.remove();
  return ok;
}

async function openReport() {
  const dlg = $("reportDlg");
  $("reportText").value = "Building the report...";
  $("reportMsg").textContent = "";
  $("reportCopy").disabled = true;
  $("reportSave").disabled = true;
  if (!dlg.open) dlg.showModal();
  try {
    $("reportText").value = await fetchReport();
    $("reportCopy").disabled = false;
    $("reportSave").disabled = false;
  } catch (e) {
    $("reportText").value = "";
    $("reportMsg").textContent = `The report could not be built: ${e.message}`;
    noteError(`Test report failed: ${e.message}`);
  }
}
$("reportBtn").addEventListener("click", openReport);

$("reportCopy").addEventListener("click", async () => {
  const text = $("reportText").value;
  if (!text) return;
  const ok = await copyText(text);
  $("reportMsg").textContent = ok ? "Copied. Paste it into a GitHub issue."
    : "Copy did not work in this browser: click into the text, press Ctrl+A, then Ctrl+C.";
});

$("reportSave").addEventListener("click", () => {
  const text = $("reportText").value;
  if (!text) return;
  const url = URL.createObjectURL(new Blob([text], { type: "text/plain;charset=utf-8" }));
  const a = h("a", { href: url, download: reportFile, class: "hidden" });
  $("reportDlg").append(a); // inside the modal dialog: the rest of the page is inert while it is open
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 5000);
  $("reportMsg").textContent = `Saved as ${reportFile} in the browser's download folder.`;
});

// The report holds the units' serial numbers and this PC's system details: it is shown before it is copied or saved,
// from here as from the Test report button.
$("logCopyReport").addEventListener("click", openReport);

// A second GUI tab shares the browser's six connections to the server: say so in both tabs.
(function watchOtherTabs() {
  if (!("BroadcastChannel" in window)) return;
  const channel = new BroadcastChannel("iaigui.tabs");
  const note = (on) => {
    $("tabNote").classList.toggle("hidden", !on);
    $("tabNote").textContent = on ? "Another GUI tab is open: its requests share this browser's connections, so STOP from either tab can wait for the other's. Use one tab." : "";
  };
  channel.onmessage = (ev) => {
    if (ev.data === "hello") {
      note(true);
      channel.postMessage("here");
    } else if (ev.data === "here") {
      note(true);
    }
  };
  channel.postMessage("hello");
})();

// ---------------------------------------------------------------- layout

// The header wraps on narrow windows; the side column's sticky offset follows its height.
(function trackHeader() {
  const bar = document.querySelector("header.appbar");
  if (!bar) return;
  const set = () => document.documentElement.style.setProperty("--appbar-h", `${Math.ceil(bar.getBoundingClientRect().height)}px`);
  set();
  if ("ResizeObserver" in window) new ResizeObserver(set).observe(bar);
  else window.addEventListener("resize", set);
})();

// ---------------------------------------------------------------- start

(async function start() {
  try {
    CATALOG = await api("/api/wb/functions");
    const bsel = $("baudRate");
    const spec = CATALOG.port.find((f) => f.name === "set_baudrate");
    for (const [r, t] of spec.inputs[0].choices) bsel.append(h("option", { value: r, text: t }));
    bsel.value = "115200";
  } catch (e) {
    toast(e.message);
  }
  await loadPorts(true);
  await poll();
  setInterval(() => {
    if (!pollRunning) poll();
  }, 250);
})();
