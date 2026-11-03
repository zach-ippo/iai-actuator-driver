// Coordinated (experimental) view: two axes along one planned path. Plan is a dry run (nothing is sent); Run sends
// a plan that passed; the plot shows the planned path and the measured trace. Built inside #viewCoord with WB.h
// (text only). STOP ALL stays with workbench.js: the header button, Space and Esc work here as on every view.
"use strict";

(function coordinatedView() {
  const WB = window.WB;
  const root = document.getElementById("viewCoord");
  if (!WB || !root) return;

  const h = WB.h;
  const $ = (id) => document.getElementById(id);
  const SVGNS = "http://www.w3.org/2000/svg";
  const STORE_KEY = "ecgui.coord.form";
  const ACTIVE = new Set(["starting", "running"]);
  const ENDED = new Set(["completed", "stopped", "refused", "failed"]);
  const MAX_TRACE = 20000;
  const FAST_MS = 200;
  const SLOW_MS = 2000;

  const has = (v) => v !== null && v !== undefined && v !== "" && Number.isFinite(Number(v));
  const f = (v, d = 2) => (has(v) ? WB.fmtN(v, d) : "?");
  const ms = (s) => (has(s) ? WB.fmtN(Number(s) * 1000, 1) : "?");
  const unitOf = (u) => (u && u.x && u.x === u.y ? u.x : "units");
  const noPeriod = (s) => String(s || "").replace(/[.\s]+$/, "");

  // ---------------------------------------------------------------- state

  let store = loadStore();
  let inited = false; // the form holds values for a pair of axes (stored or defaults)
  let initWaits = 0;
  let axesSig = "";
  let rangesSig = "";
  let readySig = "";
  let valuesFor = ""; // the "x|y" pair the position fields were filled for
  let lastPlan = null; // the last plan response (passed or refused)
  let planSig = null; // the inputs (request JSON) that produced lastPlan
  let planBody = null;
  let planError = null;
  let planAt = 0;
  let planGen = 0;
  let planVersion = 0;
  let planning = false;
  let starting = false;
  let preparing = false;
  let consumed = false; // the passed plan was used by a run: a new Plan is needed
  let runState = "idle";
  let phase = null;
  let liveTrace = true; // false: the driver reports the trace when the run ends (the core's run)
  let elapsed = 0;
  let plannedS = 0;
  let livePos = null; // [x, y] from the run status while a run is active
  let since = 0;
  let trace = []; // [t, x, y, phase] (phase 0 lead-in, 1 path)
  let traceRun = null;
  let dismissedRun = null; // a finished run whose trace a newer plan replaced
  let shownKey = null;
  let timer = null;
  let fetching = false;

  function loadStore() {
    try {
      const s = localStorage.getItem(STORE_KEY);
      const v = s ? JSON.parse(s) : null;
      return v && typeof v === "object" ? v : null;
    } catch {
      return null;
    }
  }

  // ---------------------------------------------------------------- form

  // Shape fields [key, label, kind]; each shape keeps its own values.
  const SHAPE_FIELDS = {
    circle: [["circX", "Center X", "posX"], ["circY", "Center Y", "posY"], ["circR", "Radius", "len"], ["circTurns", "Turns", "turns"]],
    line: [["lineX", "End X", "posX"], ["lineY", "End Y", "posY"]],
    rectangle: [["rectX", "Corner X", "posX"], ["rectY", "Corner Y", "posY"], ["rectW", "Width", "signedX"], ["rectH", "Height", "signedY"]],
    polygon: [["polyX", "Center X", "posX"], ["polyY", "Center Y", "posY"], ["polyR", "Radius", "len"], ["polySides", "Sides", "sides"]],
  };
  const SHAPES = [["circle", "Circle"], ["line", "Line from the current position"], ["rectangle", "Rectangle"], ["polygon", "Regular polygon"]];
  const F = {}; // key -> {input, name, hint, label, kind, range}

  function numField(key, label, kind, id) {
    const input = h("input", { type: "text", id, name: key, inputmode: "decimal", autocomplete: "off", spellcheck: "false",
      oninput: () => { markField(key); changed(); } });
    const name = h("span", { class: "coord-lab", text: label });
    const hint = h("span", { class: "range" });
    F[key] = { input, name, hint, label, kind, range: {} };
    return h("label", { class: "coord-field" }, name, input, hint);
  }
  const field = (label, control, cls = "") => h("label", { class: `coord-field ${cls}` }, h("span", { class: "coord-lab", text: label }), control);

  const selX = h("select", { id: "coordX", name: "x", onchange: onAxisChange });
  const selY = h("select", { id: "coordY", name: "y", onchange: onAxisChange });
  const selShape = h("select", { id: "coordShape", name: "shape", onchange: onShapeChange },
    SHAPES.map(([v, t]) => h("option", { value: v, text: t })));
  const selDir = h("select", { id: "coordDir", name: "dir", onchange: changed },
    h("option", { value: "ccw", text: "counter-clockwise" }), h("option", { value: "cw", text: "clockwise" }));

  const shapeBoxes = {};
  for (const [kind, list] of Object.entries(SHAPE_FIELDS)) {
    const kids = list.map(([k, l, t]) => numField(k, l, t));
    if (kind === "circle") kids.push(field("Direction", selDir, "span2"));
    shapeBoxes[kind] = h("div", { class: "coord-fields", dataset: { kind } }, kids);
  }

  const fieldset = h("fieldset", { class: "coord-fieldset", "aria-label": "Axes, shape and motion" },
    h("div", { class: "coord-fields pair" }, field("X axis", selX), field("Y axis", selY)),
    h("div", { class: "coord-fields" }, field("Shape", selShape, "wide")),
    Object.values(shapeBoxes),
    h("div", { class: "coord-fields" },
      numField("speed", "Speed", "speed", "coordSpeed"),
      numField("accel", "Accel", "accel", "coordAccel"),
      numField("chord", "Chord error", "chord", "coordChord")));

  const readyBox = h("div", { class: "coord-ready", id: "coordReady", "aria-live": "polite" });
  const prepBtn = h("button", { type: "button", id: "coordPrepare", class: "hidden", text: "Servo ON and home X and Y",
    onclick: doPrepare });
  const formErr = h("p", { class: "error hidden", role: "alert" });
  const planBtn = h("button", { type: "submit", id: "coordPlan", class: "primary", text: "Plan (dry run)",
    title: "Check the path against both axes' ranges and the line timing. Nothing is sent." });
  const runBtn = h("button", { type: "button", id: "coordRun", text: "Run", disabled: true, onclick: doRun });
  const stopBtn = h("button", { type: "button", id: "coordStop", class: "stop-mini", text: "STOP ALL",
    title: "Decelerate every connected axis to a stop (also Space or Esc)",
    onclick: async () => { await WB.stopAll(); schedule(0); } });
  const runNote = h("p", { class: "small coord-runnote", "aria-live": "polite" });
  const defaultsBtn = h("button", { type: "button", class: "mini", text: "Defaults",
    title: "A small circle through the current position; speed and accel from both axes' limits",
    onclick: () => { applyDefaults(true); showShape(); changed(); } });

  const form = h("form", { id: "coordForm", class: "coord-form", novalidate: true,
    onsubmit: (ev) => { ev.preventDefault(); doPlan(); } },
    fieldset,
    h("div", { class: "coord-readiness" }, h("div", { class: "label", text: "Readiness" }), readyBox, prepBtn),
    formErr,
    h("div", { class: "buttons coord-actions" }, planBtn, runBtn, stopBtn),
    runNote);

  // ---------------------------------------------------------------- layout

  const apiBtn = h("button", { type: "button", class: "mini toggle", id: "coordApi", "aria-pressed": "false", text: "API",
    title: "Show the Python call and the planner's summary",
    onclick: () => {
      const t = $("apiToggle");
      if (t) t.click();
      else document.body.classList.toggle("show-api");
      syncApi();
    } });
  const syncApi = () => apiBtn.setAttribute("aria-pressed", document.body.classList.contains("show-api") ? "true" : "false");
  new MutationObserver(syncApi).observe(document.body, { attributes: true, attributeFilter: ["class"] });
  syncApi();

  const addPairBtn = h("button", { type: "button", class: "primary", text: "Add two simulated axes", onclick: addPair });
  const empty = h("section", { class: "card panel coord-empty hidden", id: "coordEmpty" },
    h("h2", { text: "Two axes needed" }),
    h("p", { text: "Coordinated motion needs two axes. Connect them, or add two simulated axes." }),
    h("p", { class: "small", text: "Two sliders on two ports: enough for the Group and Coordinated tabs. Simulated axes never open a serial port." }),
    addPairBtn);

  const planOut = h("section", { class: "card panel coord-out hidden", id: "coordPlanOut", "aria-live": "polite" });
  const runOut = h("section", { class: "card panel coord-out hidden", id: "coordRunOut", "aria-live": "polite" });

  const svg = document.createElementNS(SVGNS, "svg");
  svg.id = "coordPlot";
  svg.setAttribute("role", "img");
  svg.setAttribute("aria-label", "XY plot: planned path, lead-in, measured trace and current position");
  const fullBox = h("input", { type: "checkbox", id: "coordFull", onchange: () => { save(); drawPlot(true); } });
  const progBar = h("div", { class: "bar" });
  const progTrack = h("div", { class: "track", role: "progressbar", "aria-label": "Run progress", "aria-valuemin": "0",
    "aria-valuemax": "100", "aria-valuenow": "0" }, progBar);
  const progText = h("span", { class: "small" });
  const prog = h("div", { class: "coord-progress hidden", id: "coordProgress" }, progTrack, progText);
  const legendPath = h("span", { class: "lg lg-path" }, h("i"), "Planned path");
  const legend = h("div", { class: "coord-legend" },
    legendPath,
    h("span", { class: "lg lg-lead" }, h("i"), "Lead-in"),
    h("span", { class: "lg lg-trace" }, h("i"), "Measured"),
    h("span", { class: "lg lg-trace-lead" }, h("i"), "Measured lead-in"),
    h("span", { class: "lg lg-start" }, h("i"), "Start"),
    h("span", { class: "lg lg-cur" }, h("i"), "Current position"),
    h("span", { class: "lg lg-stroke" }, h("i"), "Stroke"));
  const plotNote = h("p", { class: "small coord-plotnote" });

  const main = h("div", { class: "coord-grid hidden", id: "coordMain" },
    h("div", { class: "coord-col" },
      h("section", { class: "card panel coord-setup" },
        h("div", { class: "pane-head" }, h("h2", { text: "Path" }), h("span", { class: "spacer" }), defaultsBtn),
        form)),
    h("div", { class: "coord-col" },
      h("section", { class: "card panel coord-plotcard" },
        h("div", { class: "pane-head" }, h("h2", { text: "XY plot" }), h("span", { class: "spacer" }),
          h("label", { class: "check", title: "Zoom out to the whole stroke of both axes" }, fullBox, "Full stroke")),
        prog, svg, legend, plotNote),
      runOut,
      planOut));

  root.append(
    h("section", { class: "card panel coord-intro" },
      h("div", { class: "pane-head" },
        h("h2", { text: "Coordinated motion" }), h("span", { class: "tag exp", text: "Experimental" }),
        h("span", { class: "spacer" }), apiBtn),
      h("p", { class: "small", text: "Experimental. The PC runs both axes along one path; the units have no shared clock. " +
        "Start slow, small and mid-stroke. STOP ALL (Space / Esc) stops the run." })),
    empty,
    main);

  // ---------------------------------------------------------------- inputs

  function readField(key) {
    const fl = F[key];
    const t = fl.input.value.trim();
    const r = fl.range || {};
    if (t === "") throw Error(`${fl.label} is required.`);
    if (!/^[-+]?(\d+\.?\d*|\.\d+)(e[-+]?\d+)?$/i.test(t)) throw Error(`${fl.label}: "${t}" is not a number.`);
    const v = Number(t);
    if (!Number.isFinite(v)) throw Error(`${fl.label}: "${t}" is not a number.`);
    if (r.int && !Number.isInteger(v)) throw Error(`${fl.label} must be a whole number.`);
    if (r.nonzero && v === 0) throw Error(`${fl.label} must not be 0.`);
    if (r.positive && v <= 0) throw Error(`${fl.label} must be above 0.`);
    if ((has(r.min) && v < r.min - 1e-9) || (has(r.max) && v > r.max + 1e-9)) throw Error(`${fl.label} must be ${r.text} (got ${t}).`);
    return v;
  }

  function markField(key) {
    const fl = F[key];
    let bad = false;
    try {
      readField(key);
    } catch {
      bad = fl.input.value.trim() !== "";
    }
    fl.input.classList.toggle("invalid", bad);
    fl.hint.classList.toggle("bad", bad);
  }

  // The smallest common range of several limit entries: the larger min, the smaller max.
  function common(list) {
    const items = list.filter((r) => r && typeof r === "object");
    const mins = items.map((r) => r.min).filter(has).map(Number);
    const maxs = items.map((r) => r.max).filter(has).map(Number);
    return { min: mins.length ? Math.max(...mins) : null, max: maxs.length ? Math.min(...maxs) : null };
  }
  const limitsOf = (ax) => (ax && ax.limits) || {};
  const between = (r, unit) => (has(r.min) && has(r.max) ? `${f(r.min)} to ${f(r.max)} ${unit}`
    : has(r.max) ? `above 0, at most ${f(r.max)} ${unit}` : "above 0");
  const strokeText = (p, unit) => (p && has(p.min) && has(p.max) ? `stroke ${f(p.min)} to ${f(p.max)} ${unit}` : "any number");

  function updateRanges() {
    const xa = WB.axisOf(selX.value), ya = WB.axisOf(selY.value);
    const ux = WB.unitsOf(xa), uy = WB.unitsOf(ya);
    const pu = ux === uy ? ux : "units";
    const lx = limitsOf(xa), ly = limitsOf(ya);
    const speed = common([lx.speed, ly.speed]);
    const accel = common([lx.accel, lx.decel, ly.accel, ly.decel]);
    // A radius larger than the larger stroke cannot stay inside both ranges (and the planner's work grows with it).
    const spans = [lx.position, ly.position].filter((p) => p && has(p.min) && has(p.max)).map((p) => Number(p.max) - Number(p.min));
    const reach = spans.length ? Math.max(...spans) : null;
    for (const [key, fl] of Object.entries(F)) {
      let r = {}, unit = "";
      if (fl.kind === "posX") { unit = ux; r = { text: strokeText(lx.position, ux) }; }
      else if (fl.kind === "posY") { unit = uy; r = { text: strokeText(ly.position, uy) }; }
      else if (fl.kind === "len") {
        unit = pu;
        r = { positive: true, max: reach, text: reach === null ? "above 0" : `above 0, at most ${f(reach)} ${pu} (the larger stroke)` };
      }
      else if (fl.kind === "signedX") { unit = ux; r = { nonzero: true, text: "not 0; negative goes the other way" }; }
      else if (fl.kind === "signedY") { unit = uy; r = { nonzero: true, text: "not 0; negative goes the other way" }; }
      else if (fl.kind === "turns") r = { min: 0.05, max: 20, text: "0.05 to 20" };
      else if (fl.kind === "sides") r = { int: true, min: 3, max: 64, text: "3 to 64" };
      else if (fl.kind === "speed") { unit = `${pu}/s`; r = { positive: true, min: speed.min, max: speed.max, text: between(speed, unit) }; }
      else if (fl.kind === "accel") { unit = "G"; r = { positive: true, min: accel.min, max: accel.max, text: between(accel, "G") }; }
      else if (fl.kind === "chord") { unit = pu; r = { min: 0.001, max: 10, text: `0.001 to 10 ${pu}` }; }
      fl.range = r;
      fl.name.textContent = unit ? `${fl.label} (${unit})` : fl.label;
      fl.hint.textContent = r.text || "";
      markField(key);
    }
  }

  function setField(key, v) {
    F[key].input.value = typeof v === "number" ? WB.fmtN(v, 2) : String(v);
  }
  function valid(key) {
    try {
      readField(key);
      return true;
    } catch {
      return false;
    }
  }

  // Defaults (owner's brief): a circle through the current position, radius min(10, 10 % of the shorter stroke),
  // speed min(20, 25 % of the smaller max speed), accel min(0.3, the smaller max accel), chord error 0.05.
  // all = false keeps the shape and the motion values that still fit (used when the axes change).
  function applyDefaults(all) {
    const xa = WB.axisOf(selX.value), ya = WB.axisOf(selY.value);
    if (!xa || !ya) return;
    updateRanges();
    const px = limitsOf(xa).position || {}, py = limitsOf(ya).position || {};
    const stroke = (p) => (has(p.min) && has(p.max) && p.max > p.min ? p.max - p.min : 100);
    const at = (ax, p) => {
      const v = ax.status && ax.status.position_mm;
      return has(v) ? Number(v) : has(p.min) && has(p.max) ? (Number(p.min) + Number(p.max)) / 2 : 0;
    };
    const X = at(xa, px), Y = at(ya, py);
    valuesFor = `${selX.value}|${selY.value}`;
    const r = Math.min(10, 0.1 * Math.min(stroke(px), stroke(py)));
    const toward = (v, p, d) => (has(p.max) && v + d > p.max ? -d : d); // the side with room
    const cx = has(px.min) && X - 2 * r < px.min ? X + r : X - r;
    setField("circX", cx); setField("circY", Y); setField("circR", r); setField("circTurns", 1);
    setField("lineX", X + toward(X, px, r)); setField("lineY", Y + toward(Y, py, r));
    setField("rectX", X); setField("rectY", Y); setField("rectW", toward(X, px, 2 * r)); setField("rectH", toward(Y, py, 2 * r));
    setField("polyX", cx); setField("polyY", Y); setField("polyR", r); setField("polySides", 6);
    const lx = limitsOf(xa), ly = limitsOf(ya);
    const speed = common([lx.speed, ly.speed]), accel = common([lx.accel, lx.decel, ly.accel, ly.decel]);
    if (all) {
      selShape.value = "circle";
      selDir.value = "ccw";
    }
    if (all || !valid("speed")) setField("speed", Math.min(20, has(speed.max) ? 0.25 * speed.max : 20));
    if (all || !valid("accel")) setField("accel", Math.min(0.3, has(accel.max) ? accel.max : 0.3));
    if (all || !valid("chord")) setField("chord", 0.05);
    updateRanges();
  }

  function showShape() {
    for (const [kind, box] of Object.entries(shapeBoxes)) box.classList.toggle("hidden", kind !== selShape.value);
  }

  // The plan request from the form; throws with a plain message before any request.
  function collect() {
    const x = selX.value, y = selY.value;
    if (!x || !y) throw Error("Pick an X axis and a Y axis.");
    if (x === y) throw Error("X and Y must be two different axes.");
    for (const n of [x, y]) if (!WB.axisOf(n)) throw Error(`${n} is not connected.`);
    const kind = selShape.value;
    const n = readField;
    let shape;
    if (kind === "line") shape = { kind, to: [n("lineX"), n("lineY")] };
    else if (kind === "circle") {
      shape = { kind, center: [n("circX"), n("circY")], radius: n("circR"), turns: n("circTurns"), ccw: selDir.value !== "cw",
        start_deg: null };
    } else if (kind === "rectangle") shape = { kind, corner: [n("rectX"), n("rectY")], width: n("rectW"), height: n("rectH") };
    else shape = { kind: "polygon", center: [n("polyX"), n("polyY")], radius: n("polyR"), sides: n("polySides"), start_deg: 0 };
    return { axes: { x, y }, shape, speed: n("speed"), accel_g: n("accel"), chord_error: n("chord") };
  }
  function currentSig() {
    try {
      return JSON.stringify(collect());
    } catch {
      return null;
    }
  }

  function save() {
    if (!inited) return;
    const vals = {};
    for (const [k, fl] of Object.entries(F)) vals[k] = fl.input.value;
    const passed = !!(lastPlan && lastPlan.passed);
    store = { x: selX.value, y: selY.value, kind: selShape.value, dir: selDir.value, f: vals, full: fullBox.checked,
      planId: passed ? lastPlan.plan_id : null, planSig: passed ? planSig : null };
    try {
      localStorage.setItem(STORE_KEY, JSON.stringify(store));
    } catch {
      // no storage (private window, blocked): the page works without it
    }
  }

  function showFormError(msg) {
    formErr.textContent = msg || "";
    formErr.classList.toggle("hidden", !msg);
  }

  function changed() {
    save();
    refreshRun();
  }
  function onShapeChange() {
    showShape();
    showFormError(null);
    changed();
  }
  function onAxisChange() {
    const same = selX.value && selX.value === selY.value;
    selX.classList.toggle("invalid", !!same);
    selY.classList.toggle("invalid", !!same);
    if (same) showFormError("X and Y must be two different axes.");
    else {
      showFormError(null);
      // the positions of a new pair; speed / accel kept when they still fit
      if (`${selX.value}|${selY.value}` !== valuesFor) applyDefaults(false);
      else updateRanges();
    }
    readySig = "";
    renderReady();
    changed();
    if (!lastPlan) drawPlot(true);
  }

  function rebuildSelects(axes) {
    const label = (a) => `${a.name} (${(a.info && a.info.model) || (a.limits && a.limits.kind) || "axis"})`;
    for (const sel of [selX, selY]) {
      const keep = sel.value;
      sel.replaceChildren(...axes.map((a) => h("option", { value: a.name, text: label(a) })));
      if (axes.some((a) => a.name === keep)) sel.value = keep;
    }
  }

  function initForm(axes) {
    const names = axes.map((a) => a.name);
    const s = store && names.includes(store.x) && names.includes(store.y) && store.x !== store.y ? store : null;
    selX.value = s ? s.x : names[0];
    selY.value = s ? s.y : names[1];
    applyDefaults(true);
    if (s) {
      if (SHAPE_FIELDS[s.kind]) selShape.value = s.kind;
      selDir.value = s.dir === "cw" ? "cw" : "ccw";
      for (const [k, v] of Object.entries(s.f || {})) if (F[k] && typeof v === "string") F[k].input.value = v;
    }
    valuesFor = `${selX.value}|${selY.value}`;
    fullBox.checked = !!(store && store.full);
    showShape();
    inited = true;
    updateRanges();
    save();
    refreshRun();
    drawPlot(true);
  }

  // ---------------------------------------------------------------- readiness

  function readiness(label, name) {
    const ax = WB.axisOf(name);
    const out = { label, name, model: "", ok: false, issues: [], prep: false };
    if (!ax) {
      out.issues.push("not connected");
      return out;
    }
    out.model = (ax.info && ax.info.model) || "";
    if (ax.busy === "coordinated run") {
      out.issues.push("in the coordinated run");
      return out;
    }
    const st = ax.status;
    if (ax.fault) out.issues.push("link fault: Recover on the Axis tab");
    if (!st) out.issues.push("no status yet");
    else {
      if (st.alarm) out.issues.push(`alarm ${Number(st.alarm_code || 0).toString(16).toUpperCase().padStart(3, "0")}: reset it on the Axis tab`);
      if (!st.servo_on) out.issues.push("servo off");
      if (!st.homed) out.issues.push("not homed");
      if (st.homing) out.issues.push("homing");
      else if (st.moving) out.issues.push("moving");
      out.prep = (!st.servo_on || !st.homed) && !st.alarm && !ax.fault && !st.homing;
    }
    if (ax.busy) out.issues.push(`busy: ${ax.busy}`);
    out.ok = !out.issues.length;
    return out;
  }

  function renderReady() {
    const rows = [readiness("X", selX.value), readiness("Y", selY.value)];
    const sig = JSON.stringify(rows) + ACTIVE.has(runState);
    if (sig === readySig) return;
    readySig = sig;
    readyBox.replaceChildren(...rows.map((r) => h("div", { class: "coord-ready-row" },
      h("b", { text: r.label }), h("span", { class: "mono", text: r.name || "-" }),
      r.model ? h("span", { class: "small", text: r.model }) : null,
      h("span", { class: `chip ${r.ok ? "ok" : r.issues[0] === "in the coordinated run" ? "" : "bad"}`,
        text: r.ok ? "ready" : r.issues.join(", ") }))));
    prepBtn.classList.toggle("hidden", !rows.some((r) => r.prep));
  }

  async function doPrepare() {
    const names = [selX.value, selY.value];
    if (!names[0] || names[0] === names[1]) return;
    const unhomed = names.filter((n) => { const a = WB.axisOf(n); return !(a && a.status && a.status.homed); });
    const ok = await WB.ask({
      title: "Servo ON and home X and Y",
      text: `Servo ON for ${names.join(" and ")}${unhomed.length ? `, then home ${unhomed.join(" and ")}` : ""}. ` +
        "Home moves each axis to its mechanical end and back.",
      okText: unhomed.length ? "Servo ON and home" : "Servo ON",
    });
    if (!ok) return;
    preparing = true;
    refreshRun();
    try {
      const errs = [];
      const a = await WB.api("/api/wb/group", { op: "servo_all", axes: names, on: true });
      for (const [n, e] of Object.entries(a.errors || {})) errs.push(`${n}: ${e}`);
      const left = unhomed.filter((n) => !(a.errors || {})[n]);
      if (left.length) {
        const b = await WB.api("/api/wb/group", { op: "home_all", axes: left });
        for (const [n, e] of Object.entries(b.errors || {})) errs.push(`${n}: ${e}`);
      }
      if (errs.length) WB.toast(errs.join("\n"));
      else WB.toast(left.length ? `Servo ON; homing ${left.join(" and ")}.` : "Servo ON.", true);
    } catch (e) {
      WB.toast(e.message);
    } finally {
      preparing = false;
      refreshRun();
      WB.poll();
    }
  }

  async function addPair() {
    addPairBtn.disabled = true;
    try {
      const v = await WB.addSimPair();
      const names = ((v && v.connected) || []).flatMap((p) => p.axes || []);
      WB.toast(names.length ? `Added ${names.join(" and ")} (simulated sliders)` : "Simulated axes added", true);
    } catch (e) {
      WB.toast(`Add two simulated axes: ${e.message}`);
    } finally {
      addPairBtn.disabled = false;
    }
  }

  // ---------------------------------------------------------------- plan

  const planNum = (id) => {
    const m = /^p(\d+)$/.exec(String(id || ""));
    return m ? Number(m[1]) : -1;
  };

  async function doPlan() {
    if (planning || ACTIVE.has(runState)) return;
    let body;
    try {
      body = collect();
    } catch (e) {
      showFormError(e.message);
      return;
    }
    showFormError(null);
    planning = true;
    planGen++;
    planBtn.textContent = "Planning...";
    refreshRun();
    try {
      const v = await WB.api("/api/wb/coord/plan", body);
      setPlan(v, body);
    } catch (e) {
      lastPlan = null;
      planSig = null;
      planBody = null;
      planError = e.message;
      planVersion++;
      if (WB.noteError) WB.noteError(`Coordinated plan failed: ${e.message}`);
      renderPlanOut();
      drawPlot(true);
    } finally {
      planning = false;
      planBtn.textContent = "Plan (dry run)";
      refreshRun();
    }
  }

  function setPlan(v, body) {
    lastPlan = v;
    planBody = body;
    planSig = JSON.stringify(body);
    planError = null;
    planAt = Date.now();
    consumed = false;
    dismissedRun = traceRun; // the previous run's trace belongs to the previous path
    trace = [];
    shownKey = null;
    runOut.replaceChildren();
    runOut.classList.add("hidden");
    planVersion++;
    save();
    renderPlanOut();
    refreshRun();
    drawPlot(true);
  }

  // A passed plan stored on the server (page reload): Run stays enabled only when the form still holds its inputs.
  function adoptPlan(plan) {
    lastPlan = plan;
    planError = null;
    planAt = 0;
    consumed = false;
    if (store && store.planId === plan.plan_id && typeof store.planSig === "string") {
      planSig = store.planSig;
      try {
        planBody = JSON.parse(planSig);
      } catch {
        planBody = null;
      }
    } else {
      planSig = null;
      planBody = null;
    }
    planVersion++;
    renderPlanOut();
    refreshRun();
    drawPlot(true);
  }

  function kvTable(rows) {
    return h("table", { class: "kv coord-kv" },
      h("tbody", {}, rows.map(([k, v]) => h("tr", {}, h("th", { text: k }), h("td", { text: v })))));
  }
  function listBlock(title, items, cls = "") {
    return h("div", { class: `coord-list ${cls}` }, h("div", { class: "label", text: title }),
      h("ul", {}, items.map((t) => h("li", { text: String(t) }))));
  }
  function suggestSpeed(v) {
    const s = v >= 10 ? Math.floor(v) : v >= 1 ? Math.floor(v * 10) / 10 : Math.floor(v * 100) / 100;
    const r = (F.speed && F.speed.range) || {};
    return has(r.min) && s < r.min ? 0 : s;
  }
  function useSpeed(s) {
    F.speed.input.value = String(s);
    markField("speed");
    changed();
    F.speed.input.focus();
  }

  function pyNum(v) {
    return String(Number(Number(v).toFixed(6)));
  }
  function pyCall(body, plan) {
    const s = body.shape;
    const pair = (p) => `(${pyNum(p[0])}, ${pyNum(p[1])})`;
    let imports = "from iai_actuator_driver.coord import Path";
    let path;
    if (s.kind === "line") {
      imports = "from iai_actuator_driver.coord import Line, Path";
      path = `Path(None, [Line({"x": ${pyNum(s.to[0])}, "y": ${pyNum(s.to[1])}})])`;
    } else if (s.kind === "circle") {
      const cur = plan && plan.current;
      let on = null;
      if (Array.isArray(cur) && has(cur[0]) && has(cur[1])) {
        const dx = cur[0] - s.center[0], dy = cur[1] - s.center[1];
        if (Math.abs(Math.hypot(dx, dy) - s.radius) <= Math.max(0.005 * s.radius, 0.01)) on = (Math.atan2(dy, dx) * 180) / Math.PI;
      }
      const tail = `turns=${pyNum(s.turns)}, ccw=${s.ccw ? "True" : "False"}`;
      if (on === null) path = `Path.circle(${pair(s.center)}, ${pyNum(s.radius)}, start_deg=0, ${tail})`;
      else {
        imports = "from iai_actuator_driver.coord import Circle, Path";
        path = `Path(None, [Circle(${pair(s.center)}, ${pyNum(s.radius)}, start_deg=${pyNum(on.toFixed(3))}, ${tail})])`;
      }
    } else if (s.kind === "rectangle") path = `Path.rectangle(${pair(s.corner)}, ${pyNum(s.width)}, ${pyNum(s.height)})`;
    else path = `Path.polygon(${pair(s.center)}, ${pyNum(s.radius)}, ${s.sides}, start_deg=${pyNum(s.start_deg || 0)})`;
    return [
      imports,
      `group = cell.coordinated(x=${JSON.stringify(body.axes.x)}, y=${JSON.stringify(body.axes.y)})`,
      `plan = group.plan(${path}, ${pyNum(body.speed)}, ${pyNum(body.accel_g)}, chord_error=${pyNum(body.chord_error)})`,
      "print(plan.summary())",
      "result = group.run_plan(plan)",
    ].join("\n");
  }

  function renderPlanOut() {
    planOut.replaceChildren();
    const v = lastPlan;
    planOut.classList.toggle("hidden", !v && !planError);
    if (planError) {
      planOut.append(h("h2", { text: "Plan" }), h("p", { class: "error", text: planError }));
      return;
    }
    if (!v) return;
    const lu = unitOf(v.units), su = `${lu}/s`;
    planOut.append(h("h2", { text: "Plan" }),
      h("div", { class: `coord-verdict ${v.passed ? "ok" : "bad"}` },
        h("b", { text: v.passed ? "Plan passed" : "Plan refused" }),
        h("span", { class: "small", text: v.passed ? "Nothing sent yet: Run sends it." : "Nothing was sent." })));
    const names = v.axes || {};
    if (v.passed) {
      const rows = [
        ["Path", `${v.shape || "path"} over X ${names.x}, Y ${names.y}`],
        ["Mode", `${v.mode}, ${v.steps} steps`],
        ["Length", `${f(v.length)} ${lu}`],
        ["Duration", `${f(v.duration_s)} s at ${f(v.path_speed)} ${su}, ramps ${f(v.accel_g)} G`],
      ];
      const lead = v.lead_in;
      if (Array.isArray(lead) && lead.length > 1) {
        const a = lead[0], b = lead[lead.length - 1];
        rows.push(["Lead-in", `${f(Math.hypot(b[0] - a[0], b[1] - a[1]))} ${lu} straight from the current position to the start, ` +
          `${f(v.lead_in_s)} s, run first`]);
        rows.push(["Total", `${f(v.planned_s)} s`]);
      }
      rows.push(["Shortest step", `${ms(v.shortest_step_s)} ms (the line carries one every ${ms(v.min_step_s)} ms)`]);
      rows.push(["Estimated deviation", `${f(v.est_deviation, 3)} ${lu} (chord error ${f(v.est_chord_error, 3)} ${lu})`]);
      planOut.append(kvTable(rows));
    } else {
      const reasons = Array.isArray(v.reasons) && v.reasons.length ? v.reasons : [v.error || "refused by the planner"];
      planOut.append(listBlock("Reasons", reasons, "bad"));
      if (has(v.max_speed) && v.max_speed > 0) {
        const s = suggestSpeed(Number(v.max_speed));
        planOut.append(h("div", { class: "coord-suggest" },
          h("span", { text: `Largest feasible speed: about ${f(v.max_speed, 1)} ${su}` }),
          s > 0 ? h("button", { type: "button", class: "mini", text: `Use ${s} ${su}`,
            title: "Set the speed field; press Plan again", onclick: () => useSpeed(s) }) : null));
      }
    }
    if (Array.isArray(v.warnings) && v.warnings.length) planOut.append(listBlock("Warnings", v.warnings, "warn"));
    if (Array.isArray(v.not_ready) && v.not_ready.length) {
      planOut.append(listBlock("Not ready now (Run checks again)", v.not_ready, "warn"));
    }
    const notes = [];
    if (planBody) notes.push(h("pre", { class: "coord-code", text: pyCall(planBody, v) }));
    if (v.summary) notes.push(h("pre", { class: "coord-code", text: v.summary }));
    else if (v.error) notes.push(h("pre", { class: "coord-code", text: v.error }));
    if (notes.length) planOut.append(h("div", { class: "api-note" }, notes));
  }

  // ---------------------------------------------------------------- run

  function isSim(name) {
    const st = WB.state() || {};
    const ax = WB.axisOf(name);
    const port = (st.ports || []).find((p) => (ax && p.name === ax.port) || (p.axes || []).includes(name));
    return !!(port && port.simulated);
  }

  function runCheck() {
    if (ACTIVE.has(runState)) {
      return { can: false, note: runState === "starting" ? "Starting: the driver checks both axes." : "Running. STOP ALL (Space / Esc) stops it." };
    }
    if (planning) return { can: false, note: "Planning." };
    if (starting) return { can: false, note: "Starting." };
    if (!lastPlan) return { can: false, note: planError ? "No plan: fix the inputs and Plan again." : "Plan first: Run needs a plan that passed." };
    if (!lastPlan.passed) return { can: false, note: "The plan was refused: change the inputs and Plan again." };
    if (consumed) return { can: false, note: "Plan again for another run." };
    const c = (WB.state() || {}).coord;
    if (c && Date.now() - planAt > 1500 && (c.state !== "planned" || c.plan_id !== lastPlan.plan_id)) {
      return { can: false, note: "This plan is out of date: press Plan again." };
    }
    if (currentSig() !== planSig) return { can: false, note: "Inputs changed: Plan again." };
    return { can: true, note: `Ready to run: about ${f(lastPlan.planned_s, 1)} s.` };
  }

  function refreshRun() {
    const active = ACTIVE.has(runState);
    const { can, note } = runCheck();
    runBtn.disabled = !can;
    runBtn.classList.toggle("primary", can);
    planBtn.classList.toggle("primary", !can);
    if (runNote.textContent !== note) runNote.textContent = note;
    fieldset.disabled = active || planning;
    planBtn.disabled = active || planning;
    defaultsBtn.disabled = active;
    prepBtn.disabled = active || preparing;
    stopBtn.classList.toggle("big", active);
    const tab = $("tabCoord");
    if (tab) tab.classList.toggle("running", active);
  }

  async function doRun() {
    if (!runCheck().can) return;
    const plan = lastPlan;
    const real = [plan.axes.x, plan.axes.y].filter((n) => !isSim(n));
    if (real.length) {
      const ok = await WB.ask({
        title: "Run the coordinated move",
        text: `${plan.axes.x} (X) and ${plan.axes.y} (Y) move along the planned ${plan.shape || "path"}: about ` +
          `${f(plan.planned_s, 1)} s${Array.isArray(plan.lead_in) && plan.lead_in.length > 1 ? ", lead-in first" : ""}. ` +
          "STOP ALL (Space / Esc) stops both.\n\nExperimental: keep a hand on the emergency stop.",
        okText: "Run",
      });
      if (!ok || !runCheck().can || lastPlan !== plan) return;
    }
    starting = true;
    refreshRun();
    try {
      const v = await WB.api("/api/wb/coord/run", { plan_id: plan.plan_id });
      consumed = true;
      track(v, plan);
    } catch (e) {
      const refused = e.kind === "refused";
      if (refused) consumed = true;
      const msg = refused ? `Refused, nothing sent: ${noPeriod(e.message)}.` : e.message;
      runOut.replaceChildren(h("h2", { text: "Run" }), h("div", { class: "coord-verdict bad" }, h("b", { text: msg })));
      runOut.classList.remove("hidden");
      shownKey = null;
      WB.toast(msg);
    } finally {
      starting = false;
      refreshRun();
    }
  }

  function track(v, plan) {
    traceRun = v.run_id || null;
    dismissedRun = null;
    trace = [];
    since = 0;
    shownKey = null;
    runState = v.state || "running";
    plannedS = Number(v.planned_s) || Number(plan.planned_s) || 0;
    elapsed = 0;
    phase = null;
    livePos = null;
    runOut.replaceChildren();
    runOut.classList.add("hidden");
    renderProgress();
    refreshRun();
    drawPlot();
    schedule(0);
  }

  function renderProgress() {
    const active = ACTIVE.has(runState);
    prog.classList.toggle("hidden", !active);
    if (!active) return;
    const total = plannedS || (lastPlan && Number(lastPlan.planned_s)) || 0;
    const frac = total > 0 ? Math.min(1, Math.max(0, elapsed / total)) : 0;
    progBar.style.width = `${(frac * 100).toFixed(1)}%`;
    progTrack.setAttribute("aria-valuenow", String(Math.round(frac * 100)));
    progText.textContent = runState === "starting" ? "Starting: the driver checks both axes."
      : `${phase === "lead_in" ? "Lead-in" : "Path"}: ${f(elapsed, 1)} s of ${f(total, 1)} s${liveTrace ? "" : ". The measured trace is shown when the run ends."}`;
  }

  function renderResult(v) {
    const r = v.result;
    const u = unitOf(v.units);
    const err = noPeriod((r && r.error) || v.error || "");
    let head, cls = "bad";
    if (v.state === "completed") {
      head = `Completed in ${f(r ? r.duration_s : v.elapsed_s)} s (planned ${f(r && has(r.planned_s) ? r.planned_s : v.planned_s)} s).`;
      cls = "ok";
    } else if (v.state === "stopped") head = `Stopped after ${f(r ? r.duration_s : v.elapsed_s)} s${err ? `: ${err}` : ""}.`;
    else if (v.state === "refused") head = `Refused, nothing sent${err ? `: ${err}` : ""}.`;
    else head = `Failed${err ? `: ${err}` : ""}.`;
    runOut.replaceChildren(h("h2", { text: `Run${v.run_id ? ` ${v.run_id}` : ""}` }),
      h("div", { class: `coord-verdict ${cls}` }, h("b", { text: head })));
    if (r) {
      const rows = [];
      rows.push(["Deviation", has(r.max_deviation)
        ? `max ${f(r.max_deviation, 3)} / mean ${f(r.mean_deviation, 3)} ${u} (estimated from status samples about ${f(r.sample_spacing_ms, 0)} ms apart)`
        : "not measured"]);
      rows.push(["Duration", `${f(r.duration_s)} s (planned ${f(r.planned_s)} s)`]);
      rows.push(["Frames", `${f(r.frames, 0)} sent, ${f(r.skipped_frames, 0)} skipped`]);
      rows.push(["Late frames", `${f(r.late_frames, 0)} (worst ${f(r.worst_late_ms, 1)} ms late)`]);
      rows.push(["Start skew", `worst ${f(r.worst_skew_ms, 1)} ms between the axes`]);
      if (r.axis) rows.push(["Axis that stopped it", String(r.axis)]);
      if (Array.isArray(r.final) && r.final.length >= 2) rows.push(["Final position", `X ${f(r.final[0], 3)}, Y ${f(r.final[1], 3)}`]);
      runOut.append(kvTable(rows));
      if (Array.isArray(r.warnings) && r.warnings.length) runOut.append(listBlock("Warnings", r.warnings, "warn"));
      if (r.summary) runOut.append(h("div", { class: "api-note" }, h("pre", { class: "coord-code", text: r.summary })));
    }
    runOut.classList.remove("hidden");
  }

  // ---------------------------------------------------------------- status polling

  function schedule(delay) {
    clearTimeout(timer);
    timer = setTimeout(tick, delay);
  }

  async function tick() {
    timer = null;
    let again = false;
    if (ACTIVE.has(runState) || WB.currentView() === "viewCoord") again = await fetchStatus();
    if (timer === null) schedule(again ? 0 : ACTIVE.has(runState) ? FAST_MS : SLOW_MS);
  }

  async function fetchStatus() {
    if (fetching) return false;
    fetching = true;
    const gen = planGen;
    try {
      const v = await WB.api(`/api/wb/coord/status?since=${since}`);
      return applyStatus(v, gen);
    } catch {
      return false; // server gone or busy: the next tick tries again
    } finally {
      fetching = false;
    }
  }

  // Returns true when the status must be read again at once (a new run: its trace from the start).
  function applyStatus(v, gen) {
    if (!v || typeof v !== "object") return false;
    const rid = v.run_id || null;
    if (rid !== traceRun) {
      traceRun = rid;
      trace = [];
      if (since !== 0) {
        since = 0;
        return true;
      }
    }
    if (!(rid && rid === dismissedRun)) {
      for (const p of v.trace || []) if (Array.isArray(p) && p.length >= 3) trace.push(p);
      if (trace.length > MAX_TRACE) trace = trace.filter((_, i) => i % 2 === 0);
    }
    if (has(v.next)) since = Number(v.next);
    const was = runState;
    runState = v.state || "idle";
    phase = v.phase || null;
    elapsed = Number(v.elapsed_s) || 0;
    liveTrace = v.live_trace !== false;
    plannedS = Number(v.planned_s) || plannedS;
    livePos = ACTIVE.has(runState) && Array.isArray(v.position) && has(v.position[0]) && has(v.position[1]) ? v.position : null;
    if (runState === "planned" && v.plan && gen === planGen && !planning && planNum(v.plan.plan_id) > planNum(lastPlan && lastPlan.plan_id)) {
      adoptPlan(v.plan);
    }
    if (ENDED.has(runState)) {
      if (lastPlan && v.plan_id && v.plan_id === lastPlan.plan_id) consumed = true;
      const key = `${runState}|${rid}|${v.error || ""}|${v.result ? 1 : 0}`;
      if (key !== shownKey && !(rid && rid === dismissedRun)) {
        shownKey = key;
        renderResult(v);
      }
    }
    if (ACTIVE.has(was) && !ACTIVE.has(runState)) WB.poll();
    renderReady();
    renderProgress();
    refreshRun();
    drawPlot();
    return false;
  }

  // ---------------------------------------------------------------- plot

  const plotEls = { traceLead: null, trace: null, cur: null };
  let plotKey = "";
  let plotDirty = true;
  let curSig = "";

  function S(tag, attrs, text) {
    const el = document.createElementNS(SVGNS, tag);
    for (const [k, v] of Object.entries(attrs || {})) if (v !== null && v !== undefined) el.setAttribute(k, String(v));
    if (text !== undefined) el.textContent = text;
    return el;
  }

  function currentXY(names) {
    if (livePos) return [Number(livePos[0]), Number(livePos[1])];
    const a = WB.axisOf(names.x), b = WB.axisOf(names.y);
    const px = a && a.status && a.status.position_mm, py = b && b.status && b.status.position_mm;
    return has(px) && has(py) ? [Number(px), Number(py)] : null;
  }

  function plotData() {
    const p = lastPlan;
    const names = p && p.axes ? p.axes : { x: selX.value, y: selY.value };
    const xa = WB.axisOf(names.x), ya = WB.axisOf(names.y);
    const units = p && p.units ? p.units : { x: WB.unitsOf(xa), y: WB.unitsOf(ya) };
    let bounds = p && p.bounds ? p.bounds : null;
    if (!bounds) {
      const px = limitsOf(xa).position, py = limitsOf(ya).position;
      bounds = { x: px ? [px.min, px.max] : null, y: py ? [py.min, py.max] : null };
    }
    const okBounds = bounds && Array.isArray(bounds.x) && Array.isArray(bounds.y) && [...bounds.x, ...bounds.y].every(has);
    return {
      names, units,
      bounds: okBounds ? { x: bounds.x.map(Number), y: bounds.y.map(Number) } : null,
      path: (p && Array.isArray(p.path) && p.path) || [],
      lead: (p && Array.isArray(p.lead_in) && p.lead_in.length > 1 && p.lead_in) || null,
      refused: !!(p && !p.passed),
      cur: currentXY(names),
      equal: units.x === units.y,
    };
  }

  function bbox(points, i, b = null) {
    let x0 = b ? b.x0 : Infinity, x1 = b ? b.x1 : -Infinity, y0 = b ? b.y0 : Infinity, y1 = b ? b.y1 : -Infinity;
    for (const p of points) {
      const x = Number(p[i]), y = Number(p[i + 1]);
      if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
      if (x < x0) x0 = x;
      if (x > x1) x1 = x;
      if (y < y0) y0 = y;
      if (y > y1) y1 = y;
    }
    return x0 <= x1 && y0 <= y1 ? { x0, x1, y0, y1 } : null;
  }
  const union = (a, b) => (!a ? b : !b ? a : { x0: Math.min(a.x0, b.x0), x1: Math.max(a.x1, b.x1), y0: Math.min(a.y0, b.y0), y1: Math.max(a.y1, b.y1) });
  const inside = (a, b) => a.x0 >= b.x0 && a.x1 <= b.x1 && a.y0 >= b.y0 && a.y1 <= b.y1;

  // A minimum span (a straight line or one point still gets an area), then a 10 % margin on each side.
  function grow(b) {
    let { x0, x1, y0, y1 } = b;
    const big = Math.max(x1 - x0, y1 - y0);
    const min = big > 0 ? big * 0.05 : 1;
    if (x1 - x0 < min) { const c = (x0 + x1) / 2; x0 = c - min / 2; x1 = c + min / 2; }
    if (y1 - y0 < min) { const c = (y0 + y1) / 2; y0 = c - min / 2; y1 = c + min / 2; }
    const mx = (x1 - x0) * 0.1, my = (y1 - y0) * 0.1;
    return { x0: x0 - mx, x1: x1 + mx, y0: y0 - my, y1: y1 + my };
  }

  function computeFrame(d, W, H) {
    const m = { l: 58, r: 16, t: 14, b: 44 };
    const pw = Math.max(40, W - m.l - m.r), ph = Math.max(40, H - m.t - m.b);
    const base = [...d.path, ...(d.lead || [])];
    if (d.bounds && (fullBox.checked || (!d.path.length && !trace.length))) base.push([d.bounds.x[0], d.bounds.y[0]], [d.bounds.x[1], d.bounds.y[1]]);
    const extra = bbox(trace, 1, d.cur ? { x0: d.cur[0], x1: d.cur[0], y0: d.cur[1], y1: d.cur[1] } : null);
    const b = bbox(base, 0) || extra || { x0: 0, x1: 1, y0: 0, y1: 1 };
    let fr = grow(b);
    if (extra && !inside(extra, fr)) fr = grow(union(b, extra));
    let { x0, x1, y0, y1 } = fr;
    let kx = pw / (x1 - x0), ky = ph / (y1 - y0);
    if (d.equal) {
      const k = Math.min(kx, ky), cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
      x0 = cx - pw / k / 2; x1 = cx + pw / k / 2; y0 = cy - ph / k / 2; y1 = cy + ph / k / 2;
      kx = ky = k;
    }
    return { m, pw, ph, x0, x1, y0, y1, X: (x) => m.l + (x - x0) * kx, Y: (y) => m.t + (y1 - y) * ky };
  }

  function niceStep(span, n) {
    const raw = span / n;
    const p = 10 ** Math.floor(Math.log10(raw));
    const q = raw / p;
    return (q < 1.5 ? 1 : q < 3.5 ? 2 : q < 7.5 ? 5 : 10) * p;
  }
  function tickList(a, b, step) {
    const out = [];
    for (let k = Math.ceil(a / step - 1e-9); k * step <= b + step * 1e-9 && out.length < 60; k++) out.push(k * step);
    return out;
  }
  function tickText(v, step) {
    const d = Math.max(0, -Math.floor(Math.log10(step) + 1e-9));
    const s = v.toFixed(d);
    return /^-0(\.0+)?$/.test(s) ? s.slice(1) : s;
  }
  const pts = (list, i, fr) => list.map((p) => `${fr.X(Number(p[i])).toFixed(1)},${fr.Y(Number(p[i + 1])).toFixed(1)}`).join(" ");

  function rebuildPlot(d, fr, W, H) {
    const { m, pw, ph } = fr;
    svg.setAttribute("viewBox", `0 0 ${W} ${H}`);
    svg.replaceChildren();
    const clip = S("clipPath", { id: "coordClip" });
    clip.append(S("rect", { x: m.l, y: m.t, width: pw, height: ph }));
    const defs = S("defs");
    defs.append(clip);
    svg.append(defs, S("rect", { class: "pl-bg", x: m.l, y: m.t, width: pw, height: ph }));

    const sx = fr.x1 - fr.x0, sy = fr.y1 - fr.y0;
    const stepX = d.equal ? niceStep(Math.max(sx, sy), 6) : niceStep(sx, 5);
    const stepY = d.equal ? stepX : niceStep(sy, 5);
    const grid = S("g", { class: "pl-grid" }), ticks = S("g", { class: "pl-ticks" });
    for (const v of tickList(fr.x0, fr.x1, stepX)) {
      const x = fr.X(v);
      grid.append(S("line", { x1: x, x2: x, y1: m.t, y2: m.t + ph }));
      ticks.append(S("text", { x, y: m.t + ph + 15, "text-anchor": "middle" }, tickText(v, stepX)));
    }
    for (const v of tickList(fr.y0, fr.y1, stepY)) {
      const y = fr.Y(v);
      grid.append(S("line", { x1: m.l, x2: m.l + pw, y1: y, y2: y }));
      ticks.append(S("text", { x: m.l - 6, y: y + 4, "text-anchor": "end" }, tickText(v, stepY)));
    }
    svg.append(grid, ticks);

    const data = S("g", { "clip-path": "url(#coordClip)" });
    if (d.bounds) {
      const ax = fr.X(d.bounds.x[0]), bx = fr.X(d.bounds.x[1]), ay = fr.Y(d.bounds.y[0]), by = fr.Y(d.bounds.y[1]);
      data.append(S("rect", { class: "pl-stroke", x: Math.min(ax, bx), y: Math.min(ay, by), width: Math.abs(bx - ax), height: Math.abs(by - ay) }));
    }
    if (d.lead) data.append(S("polyline", { class: "pl-lead", points: pts(d.lead, 0, fr) }));
    if (d.path.length > 1) data.append(S("polyline", { class: d.refused ? "pl-preview" : "pl-path", points: pts(d.path, 0, fr) }));
    plotEls.traceLead = S("polyline", { class: "pl-trace-lead", points: "" });
    plotEls.trace = S("polyline", { class: "pl-trace", points: "" });
    data.append(plotEls.traceLead, plotEls.trace);
    if (d.path.length) {
      const x = fr.X(Number(d.path[0][0])), y = fr.Y(Number(d.path[0][1]));
      data.append(S("rect", { class: "pl-start", x: x - 5, y: y - 5, width: 10, height: 10 }));
    }
    plotEls.cur = S("g", { class: "pl-cur" });
    plotEls.cur.append(S("circle", { r: 6 }), S("line", { x1: -11, x2: 11, y1: 0, y2: 0 }), S("line", { x1: 0, x2: 0, y1: -11, y2: 11 }));
    data.append(plotEls.cur);
    svg.append(data, S("rect", { class: "pl-frame", x: m.l, y: m.t, width: pw, height: ph }));
    svg.append(S("text", { class: "pl-axis", x: m.l + pw / 2, y: H - 8, "text-anchor": "middle" },
      `X: ${d.names.x || "-"} (${d.units.x})`));
    svg.append(S("text", { class: "pl-axis", x: 0, y: 0, "text-anchor": "middle", transform: `translate(15 ${m.t + ph / 2}) rotate(-90)` },
      `Y: ${d.names.y || "-"} (${d.units.y})`));
  }

  function updateDynamic(d, fr) {
    let lead = "", path = "", last = "";
    for (const p of trace) {
      const s = `${fr.X(Number(p[1])).toFixed(1)},${fr.Y(Number(p[2])).toFixed(1)}`;
      if (Number(p[3]) === 0) {
        lead += `${s} `;
        last = s;
      } else {
        if (!path && last) path = `${last} `; // join the lead-in to the path
        path += `${s} `;
      }
    }
    plotEls.traceLead.setAttribute("points", lead.trim());
    plotEls.trace.setAttribute("points", path.trim());
    if (d.cur) {
      plotEls.cur.setAttribute("transform", `translate(${fr.X(d.cur[0]).toFixed(1)} ${fr.Y(d.cur[1]).toFixed(1)})`);
      plotEls.cur.removeAttribute("display");
    } else plotEls.cur.setAttribute("display", "none");
  }

  function renderPlotNote(d) {
    legendPath.className = `lg ${d.refused ? "lg-preview" : "lg-path"}`;
    legendPath.lastChild.textContent = d.refused ? "Refused path (preview)" : "Planned path";
    const parts = [];
    if (!lastPlan) parts.push("Press Plan to draw the path. Nothing moves before Run.");
    else if (d.refused) parts.push("Refused: the plot shows the asked shape; nothing was sent.");
    if (!d.equal) parts.push("The axes have different units: each axis has its own scale.");
    const text = parts.join(" ");
    if (plotNote.textContent !== text) plotNote.textContent = text;
  }

  function drawPlot(force = false) {
    if (root.classList.contains("hidden") || main.classList.contains("hidden")) {
      plotDirty = true;
      return;
    }
    const rect = svg.getBoundingClientRect();
    const W = Math.round(rect.width), H = Math.round(rect.height);
    if (W < 60 || H < 60) {
      plotDirty = true;
      return;
    }
    const d = plotData();
    const fr = computeFrame(d, W, H);
    const key = [W, H, fr.x0, fr.x1, fr.y0, fr.y1].map((v) => Number(v).toPrecision(7)).join(",") +
      `|${planVersion}|${fullBox.checked}|${d.names.x}|${d.names.y}|${d.units.x}|${d.units.y}|${d.bounds ? d.bounds.x.concat(d.bounds.y).join(",") : ""}`;
    if (force || plotDirty || key !== plotKey || !plotEls.cur) {
      rebuildPlot(d, fr, W, H);
      plotKey = key;
      plotDirty = false;
    }
    updateDynamic(d, fr);
    renderPlotNote(d);
  }

  if (window.ResizeObserver) new ResizeObserver(() => drawPlot()).observe(svg);
  else window.addEventListener("resize", () => drawPlot());

  // ---------------------------------------------------------------- page hooks

  WB.onState((st) => {
    const axes = (st && st.axes) || [];
    const sig = axes.map((a) => `${a.name}|${(a.info && a.info.model) || ""}`).join(",");
    if (sig !== axesSig) {
      axesSig = sig;
      const before = [selX.value, selY.value];
      rebuildSelects(axes);
      if (inited && axes.length >= 2 && (selX.value !== before[0] || selY.value !== before[1] || selX.value === selY.value)) {
        if (selX.value === selY.value) selY.value = (axes.find((a) => a.name !== selX.value) || {}).name || "";
        onAxisChange();
      }
    }
    const enough = axes.length >= 2;
    empty.classList.toggle("hidden", enough);
    main.classList.toggle("hidden", !enough);
    if (enough && !inited) {
      const names = axes.map((a) => a.name);
      const s = store && names.includes(store.x) && names.includes(store.y) ? [store.x, store.y] : names.slice(0, 2);
      const known = s.every((n) => { const a = WB.axisOf(n); return a && a.limits && a.limits.position && a.status; });
      if (known || ++initWaits > 20) initForm(axes);
    }
    if (inited) {
      const xa = WB.axisOf(selX.value), ya = WB.axisOf(selY.value);
      const lsig = JSON.stringify([selX.value, selY.value, xa && xa.limits, ya && ya.limits].map((v) => v || null));
      if (lsig !== rangesSig) {
        rangesSig = lsig;
        updateRanges();
      }
    }
    renderReady();
    const c = st && st.coord;
    if (c && ACTIVE.has(c.state) && !ACTIVE.has(runState) && !fetching) schedule(0); // a run started elsewhere: follow it
    refreshRun();
    if (!livePos) {
      const d = currentXY(lastPlan && lastPlan.axes ? lastPlan.axes : { x: selX.value, y: selY.value });
      const cs = d ? `${d[0].toFixed(3)},${d[1].toFixed(3)}` : "";
      if (cs !== curSig) {
        curSig = cs;
        drawPlot();
      }
    }
  });

  WB.onView((id) => {
    if (id !== "viewCoord") return;
    drawPlot(true);
    schedule(0);
  });

  // ---------------------------------------------------------------- start

  showShape();
  refreshRun();
  (async () => {
    await fetchStatus(); // restore a stored plan, the last result or a run in progress
    if (timer === null) schedule(ACTIVE.has(runState) ? FAST_MS : SLOW_MS);
  })();
})();
