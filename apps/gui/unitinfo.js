"use strict";
// Unit info / diagnostics view. READ-ONLY: it only sends GET requests -
//   /api/unit_info   everything the EC tells about itself (FC03 + FC11 reads, on Refresh)
//   /api/unit_live   the ~5 Hz status poll's words decoded by the server (no extra bus traffic)
//   /api/read_block  raw words of one directory block (register browser, FC03 inside the directory)
// Nothing in this file can write to the EC. STOP stays in the sticky header (app.js). The user-parameter
// Edit buttons are built by params.js (which writes, after a confirmation); this file only places them.
// Uses app.js globals: $, el, api, toast, last.

const INFO_LIVE_MS = 200; // the server polls status at ~5 Hz
const INFO_SECTIONS = ["secIdentity", "secParams", "secIo", "secStatus", "secMonitor", "secAlarms", "secMaint",
  "secPositions", "secBrowser"];
// Monitor block members (0x9000, register order): address and a short description.
const WORDS = {
  PNOW: ["0x9000", "current position (0.01 mm)"],
  ALMC: ["0x9002", "alarm code"],
  DIPM: ["0x9003", "PIO input image"],
  DOPM: ["0x9004", "PIO output image"],
  DSS1: ["0x9005", "device status 1"],
  DSS2: ["0x9006", "device status 2"],
  DSSE: ["0x9007", "expanded device status"],
  STAT: ["0x9008", "system status (32 bit)"],
  VNOW: ["0x900A", "current speed"],
  CNOW: ["0x900C", "motor current"],
  DEVI: ["0x900E", "deviation"],
  STIM: ["0x9010", "time since power-on (ms)"],
  SIPM: ["0x9012", "special input image"],
  ZONS: ["0x9013", "zone / LS status"],
  POSS: ["0x9014", "position complete number"],
  SSSE: ["0x9015", "special status"],
  OLLV: ["0x9020", "overload level"],
};
// Lamp colour when a bit is 1 (default green); active-low bits are green at 1 and red at 0.
const BIT_KIND = {
  EMGS: "alert", ALMH: "alert", ABER: "alert", EMGP: "alert", MPUV: "alert", EMG: "alert",
  ALML: "warn", PSFL: "warn", SFTY: "warn", STP: "warn", BATL: "warn", OVLW: "warn",
};

let info = null; // last /api/unit_info result (null: not read for this connection)
let infoVersion = 0; // bumped on every new read: refresh-only tables inside live sections rebuild once per read
let liveData = null; // last /api/unit_live result
let infoConnKey = ""; // connection the data belongs to
let refreshing = false;
let liveBusy = false;
let browseBlock = null; // last /api/read_block result

// ------------------------------------------------------------------ small helpers

const isNum = (v) => typeof v === "number" && Number.isFinite(v);
const hexN = (v, digits = 4) => (isNum(v) ? "0x" + v.toString(16).toUpperCase().padStart(digits, "0") : "—");
const fix2 = (v) => (isNum(v) ? v.toFixed(2) : "—");
const mmText = (v) => (isNum(v) ? `${v.toFixed(2)} mm` : "—");
const pairText = (p, unit) => (Array.isArray(p) && p.length === 2 ? `${fix2(p[0])} – ${fix2(p[1])} ${unit}` : "—");
const show = (v) => (v === null || v === undefined || v === "" ? "—" : typeof v === "object" ? JSON.stringify(v) : String(v));

function cell(content, cls, title) {
  const td = el("td");
  if (content instanceof Node) td.append(content);
  else td.textContent = show(content);
  if (cls) td.className = cls;
  if (title) td.title = title;
  return td;
}

// headers: ["Label" | ["Label", "tooltip"]]; rows: arrays of cell contents ({v, cls, title} or plain)
function table(headers, rows, cls = "tbl") {
  const t = el("table", undefined, cls);
  const head = el("tr");
  for (const h of headers) {
    const th = el("th", Array.isArray(h) ? h[0] : h);
    if (Array.isArray(h)) th.title = h[1];
    head.append(th);
  }
  const thead = el("thead");
  thead.append(head);
  const body = el("tbody");
  for (const row of rows) {
    const tr = el("tr");
    if (row.title) tr.title = row.title;
    if (row.cls) tr.className = row.cls;
    for (const c of row.cells || row) {
      tr.append(c && typeof c === "object" && !(c instanceof Node) ? cell(c.v, c.cls, c.title) : cell(c));
    }
    body.append(tr);
  }
  t.append(thead, body);
  const wrap = el("div", undefined, "tbl-wrap");
  wrap.append(t);
  return wrap;
}

// pairs: [label, value, title?]
function kvTable(pairs) {
  const t = el("table", undefined, "kv");
  const body = el("tbody");
  for (const [k, v, title] of pairs) {
    const tr = el("tr");
    tr.append(el("th", k), cell(v, undefined, title));
    if (title) tr.title = title;
    body.append(tr);
  }
  t.append(body);
  return t;
}

function heading(text, extra) {
  const h = el("div", text, "label sub-head");
  if (extra) h.append(el("span", " " + extra, "small"));
  return h;
}

// Directory record layout: members x entries x member size (MS 0 = members of mixed sizes).
function layoutText(members, entries, size, long = false) {
  const each = size ? `${size} word${size > 1 ? "s" : ""}` : "mixed sizes";
  return long ? `${members} members × ${entries} entries, ${each} each` : `${members} × ${entries} × ${size || "mixed"}`;
}

function note(text) {
  return el("p", text, "small");
}

function placeholder(text) {
  return el("p", text, "small placeholder");
}

// ------------------------------------------------------------------ bit lamps (live, updated in place)

function bitTitle(word, b) {
  const lines = [`${word} b${b.bit} ${b.name}${b.meaning ? " - " + b.meaning : ""}`];
  lines.push((b.confirmed ? "Confirmed on hardware: " : "Unconfirmed: ") + (b.evidence || ""));
  if (b.active_low) lines.push("Active low: 1 = normal (green), 0 = active (red)");
  return lines.join("\n");
}

function bitLamp(word, b) {
  const lamp = el("div", undefined, "lamp bit");
  lamp.dataset.bit = String(b.bit);
  lamp.dataset.kind = BIT_KIND[b.name] || "ok";
  lamp.dataset.unconfirmed = b.confirmed ? "" : "1";
  if (b.active_low) lamp.dataset.activeLow = "1";
  const label = el("span");
  label.append(el("small", `b${b.bit}`), document.createTextNode(" " + b.name + (b.confirmed ? "" : "?")));
  lamp.append(el("i"), label);
  lamp.title = bitTitle(word, b);
  return lamp;
}

function setLamp(lamp, on) {
  let cls = "lamp bit" + (lamp.dataset.unconfirmed ? " unconfirmed" : "");
  if (on !== true && on !== false) cls += " unknown";
  else if (lamp.dataset.activeLow) cls += on ? " on ok" : " on alert";
  else cls += on ? " on " + lamp.dataset.kind : " off";
  if (lamp.className !== cls) lamp.className = cls;
}

// One status / I/O word: header (name, value, address, source) and a lamp per bit.
function wordSlot(container, name) {
  let block = container.querySelector(`:scope > .word[data-word="${name}"]`);
  if (!block) {
    block = el("div", undefined, "word");
    block.dataset.word = name;
    const head = el("div", undefined, "word-head");
    head.append(el("b", name), el("code", "—"), el("span", "", "small"));
    block.append(head, el("div", undefined, "lamps bits"));
    container.append(block);
  }
  return block;
}

function renderWord(container, name, decoded, source, label) {
  const block = wordSlot(container, name);
  const [, value, desc] = block.querySelector(".word-head").children;
  const addr = WORDS[name] ? WORDS[name][0] : "";
  const what = label || (WORDS[name] ? WORDS[name][1] : "");
  desc.textContent = [addr, what, source].filter(Boolean).join(" · ");
  const lamps = block.querySelector(".lamps");
  if (!decoded) {
    value.textContent = "—";
    const known = lamps.querySelectorAll(".lamp");
    for (const lamp of known) setLamp(lamp, undefined);
    if (!known.length && !lamps.children.length) lamps.replaceChildren(placeholder("Not read yet."));
    return;
  }
  value.textContent = decoded.hex;
  const sig = decoded.bits.map((b) => `${b.bit}:${b.name}`).join(",");
  if (lamps.dataset.sig !== sig) {
    lamps.dataset.sig = sig;
    lamps.replaceChildren(...decoded.bits.map((b) => bitLamp(name, b)));
  }
  decoded.bits.forEach((b, i) => setLamp(lamps.children[i], b.on));
  block.title = decoded.unknown_set && decoded.unknown_set.length
    ? `Set bits without a name: ${decoded.unknown_set.map((b) => "b" + b).join(", ")}` : "";
}

// ------------------------------------------------------------------ live data (status poll) merged with the last read

function liveOr(section, name) {
  const live = liveData && liveData.live ? liveData.live[section] : null;
  if (live && live[name] !== undefined) return [live[name], "live"];
  const read = info ? info[section] : null;
  if (read && read[name] !== undefined) return [read[name], "last Refresh"];
  return [null, ""];
}

function currentMode() {
  const live = liveData && liveData.live ? liveData.live.position_mode : null;
  const read = info && info.identity ? info.identity.position_mode : null;
  return read || live || null;
}

function modeText(mode) {
  if (!mode) return "position mode unknown";
  const why = mode.confirmed ? "" : ` (unconfirmed${mode.note ? ": " + mode.note : ""})`;
  return `${mode.name} - from ${mode.source}${why}`;
}

function renderIo() {
  const box = $("infoIo");
  if (!box.dataset.built) {
    box.dataset.built = "1";
    box.replaceChildren(
      el("p", "", "small io-mode"),
      heading("Position lamps", "ZONS b12 / b14 / b13 (same on every unit tested); alarm = ALMC ≠ 0 or DSS1 ALMH"),
      el("div", undefined, "lamps io-lamps"),
      heading("Inputs and outputs"),
      el("div", undefined, "io-words"),
      el("p", "", "small io-note"),
    );
    const lampBox = box.querySelector(".io-lamps");
    for (const [key, label, kind] of [["backward_end", "Backward end", "ok"], ["middle", "Middle", "ok"],
      ["forward_end", "Forward end", "ok"], ["alarm", "Alarm", "alert"]]) {
      const lamp = el("div", undefined, "lamp unknown");
      lamp.dataset.key = key;
      lamp.dataset.kind = kind;
      lamp.append(el("i"), el("span", label));
      lampBox.append(lamp);
    }
  }
  const mode = currentMode();
  const src = liveData && liveData.live ? "" : " (Refresh / connect for live values)";
  box.querySelector(".io-mode").textContent =
    `Position mode: ${modeText(mode)}. DIPM / DOPM bit names follow this mode${src}.` +
    (liveData && liveData.mode_source && !(info && info.identity) ? ` Mode source: ${liveData.mode_source}.` : "");
  const [lamps] = liveOr("io", "lamps");
  for (const lamp of box.querySelectorAll(".io-lamps .lamp")) {
    const v = lamps ? lamps[lamp.dataset.key] : undefined;
    lamp.className = "lamp " + (v === true ? "on " + lamp.dataset.kind : v === false ? "off" : "unknown");
  }
  const words = box.querySelector(".io-words");
  const labels = {
    DIPM: "PIO inputs (names from the EC manuals; ST0 / ST1 of the 2-position layout VERIFIED-LIVE on the gripper, the rest unconfirmed)",
    DOPM: "PIO outputs (layout per position mode, VERIFIED-LIVE on both units)",
    SIPM: "special inputs",
    ZONS: "zones Z1 / Z2 and LS0-LS2",
  };
  for (const name of ["DIPM", "DOPM", "SIPM", "ZONS"]) {
    const [decoded, source] = liveOr("io", name);
    renderWord(words, name, decoded, source, `${WORDS[name][1]}: ${labels[name]}`);
  }
  const [ioNote] = liveOr("io", "note");
  box.querySelector(".io-note").textContent = ioNote || "";
  const [dipm] = liveOr("io", "DIPM");
  const [dopm] = liveOr("io", "DOPM");
  $("sumIo").textContent = mode ? `${mode.name} · DIPM ${dipm ? dipm.hex : "—"} · DOPM ${dopm ? dopm.hex : "—"}` : "";
}

function renderStatus() {
  const box = $("infoStatus");
  if (!box.dataset.built) {
    box.dataset.built = "1";
    box.replaceChildren(
      el("div", undefined, "status-words"),
      el("p", "", "small poss-line"),
      heading("Control words (0x0D00 readback)", "as of the last Refresh; this view never writes them"),
      el("div", undefined, "control-words"),
      el("div", undefined, "control-other"),
    );
  }
  const words = box.querySelector(".status-words");
  for (const name of ["DSS1", "DSS2", "DSSE", "ZONS", "STAT", "SSSE", "SIPM"]) {
    const [decoded, source] = liveOr("status_words", name);
    renderWord(words, name, decoded, source || (name === "SSSE" ? "Refresh to read (not in the status poll)" : ""));
  }
  const [poss] = liveOr("status_words", "POSS");
  box.querySelector(".poss-line").textContent = poss
    ? `POSS 0x9014 = ${poss.hex}: position complete number ${poss.position_no} (${poss.evidence}; last Refresh)`
    : "POSS 0x9014: Refresh to read (not in the status poll).";
  const control = info ? info.control_words : null;
  const cw = box.querySelector(".control-words");
  const other = box.querySelector(".control-other");
  if (cw.dataset.version === String(infoVersion)) {
    // refresh-only content: already drawn for this read
  } else if (!control) {
    cw.dataset.version = String(infoVersion);
    cw.replaceChildren(placeholder(info ? "Not readable (see errors above)." : "Press Refresh to read."));
    cw.dataset.built = "";
    other.replaceChildren();
  } else {
    cw.dataset.version = String(infoVersion);
    if (!cw.dataset.built) cw.replaceChildren();
    cw.dataset.built = "1";
    for (const name of ["DRG1", "DRG2", "DRGE"]) {
      if (control[name]) renderWord(cw, name, control[name], "last Refresh", `${control[name].address} control register`);
    }
    const rest = Object.entries(control).filter(([k]) => !["DRG1", "DRG2", "DRGE"].includes(k));
    other.replaceChildren(table(["Member", "Address", "Value"], rest.map(([k, v]) => [k, v.address, { v: v.hex, cls: "mono" }])));
  }
  const [dss1] = liveOr("status_words", "DSS1");
  const [dsse] = liveOr("status_words", "DSSE");
  $("sumStatus").textContent = dss1 ? `DSS1 ${dss1.hex} (${dss1.on.join(" ") || "none"}) · DSSE ${dsse ? dsse.hex : "—"}` : "";
}

const MONITOR_TILES = [
  // [id, label, source word, formatter(monitor, alarm) -> [value, unit, small]]
  ["position", "Position", "PNOW", (m) => [fix2(m.position_mm), "mm", ""]],
  ["speed", "Speed", "VNOW", (m) => [fix2(m.velocity_mm_s), "mm/s", "quantized ~12.5 mm/s: not for control"]],
  ["current", "Motor current", "CNOW", (m) => [show(m.current), isNum(m.current_pct) ? `${m.current_pct} % of rated` : "",
    isNum(m.rated_current) ? `CNOW / MCUR (${m.rated_current}) x 100 (scale unverified)` : "unit unconfirmed"]],
  ["deviation", "Deviation", "DEVI", (m) => [show(m.deviation), "pulses", "DOC-CON"]],
  ["uptime", "Uptime", "STIM", (m) => [show(m.uptime), "", isNum(m.uptime_ms) ? `${m.uptime_ms} ms since power-on` : ""]],
  ["alarm", "Alarm code", "ALMC", (m, a) => [isNum(m.alarm_code) ? hexN(m.alarm_code, 3) : "—", "", a ? a.name : ""]],
  ["overload", "Overload level", "OLLV", (m) => [show(m.overload_level), "", "probably % (unconfirmed)"]],
  ["poss", "Position complete no.", "POSS", (m) => [show(m.position_complete_no), "", "DOC-CON"]],
];

function renderMonitor() {
  const box = $("infoMonitor");
  if (!box.dataset.built) {
    box.dataset.built = "1";
    const tiles = el("div", undefined, "tiles");
    for (const [id, label, word] of MONITOR_TILES) {
      const t = el("div", undefined, "card stat mini");
      t.dataset.tile = id;
      const lab = el("div", `${label} · ${word} ${WORDS[word][0]}`, "label");
      const strong = el("strong", "—");
      strong.append(el("span", ""));
      t.append(lab, strong, el("div", "", "small"));
      tiles.append(t);
    }
    box.replaceChildren(tiles, el("div", undefined, "monitor-other"), el("p", "", "small mono monitor-raw"),
      el("ul", undefined, "small monitor-notes"));
  }
  const live = liveData && liveData.live ? liveData.live.monitor : null;
  const read = info ? info.monitor : null;
  const m = { ...(read || {}), ...(live || {}) };
  const alarm = liveData && liveData.live && liveData.live.alarm ? liveData.live.alarm
    : info && info.alarms && info.alarms.current ? info.alarms.current : null;
  for (const [id, , word, fmt] of MONITOR_TILES) {
    const t = box.querySelector(`[data-tile="${id}"]`);
    const [value, unit, small] = fmt(m, alarm);
    const strong = t.querySelector("strong");
    strong.firstChild.textContent = value;
    strong.querySelector("span").textContent = unit;
    const isLive = live && live.raw && live.raw[word] !== undefined;
    t.querySelector(".small").textContent = small + (isLive ? "" : read ? " (last Refresh)" : "");
    t.classList.toggle("stale", !isLive);
    t.classList.toggle("alert", id === "alarm" && isNum(m.alarm_code) && m.alarm_code !== 0);
  }
  const other = box.querySelector(".monitor-other");
  if (other.dataset.version !== String(infoVersion)) {
    other.dataset.version = String(infoVersion);
    if (read && read.other) {
      other.replaceChildren(heading("Other monitor values", "last Refresh; IAI symbol names"),
        table(Object.keys(read.other), [Object.values(read.other).map((v) => ({ v, cls: "mono" }))]));
    } else other.replaceChildren();
  }
  const raw = { ...((read && read.raw) || {}), ...((live && live.raw) || {}) };
  box.querySelector(".monitor-raw").textContent = Object.keys(raw).length
    ? "Raw: " + Object.entries(raw).map(([k, v]) => `${k} ${v}`).join(" · ") : "";
  const notes = m.notes || {};
  const ul = box.querySelector(".monitor-notes");
  const key = JSON.stringify(notes);
  if (ul.dataset.key !== key) {
    ul.dataset.key = key;
    ul.replaceChildren(...Object.values(notes).map((n) => el("li", n)));
  }
  const age = liveData && isNum(liveData.status_age_s) ? ` · status ${liveData.status_age_s.toFixed(1)} s old` : "";
  $("sumMonitor").textContent = Object.keys(m).length > 2
    ? `${fix2(m.position_mm)} mm · up ${show(m.uptime)}${age}${liveData && liveData.comm_ok === false ? " · NO RESPONSE" : ""}` : "";
}

function renderAlarmNow() {
  const box = $("alarmNow");
  if (!box) return;
  const live = liveData && liveData.live ? liveData.live.alarm : null;
  const read = info && info.alarms ? info.alarms.current : null;
  const cur = live || read;
  if (!cur) {
    box.className = "alarm-now idle";
    box.replaceChildren(el("b", "Current alarm: —"));
    return;
  }
  const active = cur.code !== 0;
  // A message-level code (0x011-0x07F) is a warning: the axis keeps running; not the red alarm.
  const warning = active && cur.fault === false;
  box.className = "alarm-now " + (active ? (warning ? "warning" : "active") : "clear");
  const parts = [el("b", active ? `${warning ? "WARNING" : "ALARM"} ${cur.hex}` : "No alarm"), el("span", ` ${cur.name || ""}`)];
  if (active && cur.level) parts.push(el("span", ` · level: ${cur.level}`, "small"));
  if (read && read.code === cur.code && active) {
    parts.push(el("span", ` · class 3 item 10: ${hexN(read.class3_code, 3)}`, "small"));
  }
  if (active && cur.remedy) parts.push(el("span", ` · ${cur.remedy}`, "small"));
  parts.push(el("span", live ? " · live (ALMC 0x9002)" : " · last Refresh", "small"));
  box.replaceChildren(...parts);
}

function renderLive() {
  renderIo();
  renderStatus();
  renderMonitor();
  renderAlarmNow();
}

// ------------------------------------------------------------------ sections read on Refresh

function renderIdentity() {
  const box = $("infoIdentity");
  const id = info && info.identity;
  if (!id) {
    box.replaceChildren(placeholder(info ? "Identity not readable (see errors above)." : "Press Refresh to read the unit."));
    $("sumIdentity").textContent = "";
    return;
  }
  const mode = id.position_mode;
  const baud = id.baudrate === null || id.baudrate === undefined
    ? "simulated (no serial port)" : `${id.baudrate} baud${id.baud_detected ? " (auto-detected)" : ""}`;
  const pairs = [
    ["Model (0x0600)", id.model],
    ["Model name", id.model_name],
    ["Unit name (0x061A)", id.user_name, "ANA1-3: a name the user can edit, not a model field"],
    ["Serial number", id.serial_number],
    ["Controller (FC11)", id.controller + (id.identity_source && id.identity_source !== "FC11" ? ` (identity from ${id.identity_source})` : "")],
    ["Actuator (AKID 7.33)", id.actuator || "—", "Actuator kind from AKID, item 7.33 (the unit code gives the family)"],
    ["Axis number (AXNO 0x8004)", isNum(id.axis_number) ? id.axis_number : "—"],
    ["System code (FC11)", isNum(id.system_code) ? `${hexN(id.system_code, 2)} ${id.system_name || ""}` : id.system_code],
    ["Unit code (FC11)", isNum(id.unit_code) ? `${hexN(id.unit_code, 2)} ${id.unit_name || ""}` : id.unit_code],
    ["Firmware (FC11)", id.firmware],
    ["Run status (FC11)", isNum(id.run_status) ? hexN(id.run_status, 2) : id.run_status],
    ["Raw slave ID (FC11)", id.slave_id_raw],
    ["Slave address", id.slave],
    ["Baud", baud],
    ["Soft limits (0x0700)", Array.isArray(id.soft_limits_mm) ? `${fix2(id.soft_limits_mm[0])} / ${fix2(id.soft_limits_mm[1])} mm` : "—"],
    ["EC stroke", pairText(id.stroke_mm, "mm"), "Soft limits minus their 0.30 mm margin"],
    ["Stroke in use", pairText(id.effective_stroke_mm, "mm")],
    ["Stroke override", id.configured_stroke_mm ? pairText(id.configured_stroke_mm, "mm") : "none (the EC's own)"],
    ["EC max speed (MAXV 7.4)", isNum(id.ec_max_speed_mm_s) ? `${fix2(id.ec_max_speed_mm_s)} mm/s` : "—", id.ec_max_speed_source || ""],
    ["Speed cap in use", isNum(id.speed_cap_mm_s) ? `${fix2(id.speed_cap_mm_s)} mm/s` : "—"],
    ["Max accel in use", isNum(id.accel_cap_g) ? `${id.accel_cap_g} G` : isNum(id.max_accel_g) ? `${id.max_accel_g} G` : "—", "min(the configured cap, MAXA / MAXD 7.5 / 7.6)"],
    ["Push cap in use", isNum(id.push_cap_pct) ? `${id.push_cap_pct} %` : "—", "min(the configured cap, MXPP 7.9)"],
    ["Position mode", mode ? modeText(mode) : "—"],
  ];
  const handled = new Set(["model", "model_name", "short_model", "serial_number", "controller", "system_code",
    "system_name", "unit_code", "unit_name", "firmware", "run_status", "slave_id_raw", "slave", "baudrate",
    "baud_detected", "soft_limits_mm", "stroke_mm", "effective_stroke_mm", "configured_stroke_mm",
    "ec_max_speed_mm_s", "max_speed_mm_s", "speed_cap_mm_s", "max_accel_g", "position_mode",
    "identification_extra", "directory", "notes", "user_name", "identity_source", "actuator", "unit_known",
    "axis_number", "ec_max_speed_source", "accel_cap_g", "push_cap_pct", "capabilities", "actuator_kind",
    "actuator_kind_name", "family", "actuator_serial", "accl", "extensibility_level", "rotary", "max_decel_g",
    "speed_limit_source"]);
  for (const [k, v] of Object.entries(id)) if (!handled.has(k)) pairs.push([k, v]);
  const extra = id.identification_extra || {};
  if (Object.keys(extra).length) {
    const accl = extra.accl || {};
    pairs.push(["Actuator serial (0x0608)", extra.actuator_serial],
      ["Manufacture info (0x0610)", extra.manufacture_info ? `${extra.manufacture_info.mrev} / ${extra.manufacture_info.frev} (MREV / FREV)` : extra.code],
      ["ACCL (0x0612)", accl.hex ? `${accl.hex}: extensibility level ${accl.extensibility_level} (${accl.level_name})` : "—"],
      ["BISD (0x0614)", extra.bisd_raw ? `${extra.bisd_raw} (date? ${extra.date_guess || "—"})` : extra.date_guess, extra.note]);
  }
  const caps = id.capabilities || {};
  const flags = (caps.support && caps.support.flags) || {};
  if (Object.keys(flags).length) {
    const on = Object.entries(flags).filter(([, v]) => v === true).map(([k]) => k.replace(/_/g, " "));
    pairs.push(["Support bits (class 1)", on.join(", ") || "—", "SUPT / MVPT / SUP2 / SUP3 (IAI bit names)"]);
  }
  if (caps.collision) {
    const items = (caps.collision.parameter_items || []).join(", ") || "none";
    pairs.push(["Collision detection", `${caps.collision.reported}; class 8 items: ${items}`, caps.collision.note]);
  }
  if (caps.axis_limits) {
    const a = caps.axis_limits;
    pairs.push(["Axis limits (class 7)",
      `MAXV ${isNum(a.max_speed_mm_s) ? fix2(a.max_speed_mm_s) : "—"} mm/s, MAXA / MAXD ${a.max_accel_g ?? "—"} / ${a.max_decel_g ?? "—"} G, ` +
      `push ${a.min_push_pct}..${a.max_push_pct ?? "—"} % (MNPP..MXPP), MXPH ${a.max_push_speed_mm_s ?? "—"} mm/s` +
      (a.push_available === false ? " - push not available" : ""), a.push_force_note || ""]);
  }
  if (caps.link && isNum(caps.link.boot_baudrate)) pairs.push(["Power-up baud (BRSL 8.3)", `${caps.link.boot_baudrate} baud`, caps.link.note]);
  if (caps.servo_monitor) pairs.push(["Servo monitor (FMNT 8.54)", `${caps.servo_monitor.mode_name} (${caps.servo_monitor.fmnt_hex})`, caps.servo_monitor.note]);
  if (isNum(caps.push_speed_mm_s)) pairs.push(["Push speed (PSHV 8.15)", `${fix2(caps.push_speed_mm_s)} mm/s`]);
  if (Array.isArray(id.notes) && id.notes.length) pairs.push(["Notes", id.notes.join("; ")]);
  const nodes = [kvTable(pairs)];
  if (extra.note) nodes.push(note("Identification extras: " + extra.note));
  const dir = id.directory;
  if (dir) {
    const h = dir.header || {};
    nodes.push(heading("Directory (index table at 0x0000)",
      `${dir.record_count} records, ${dir.total_words} registers · model code ${show(h.model_code)}, ` +
      `${show(h.words_per_record)} words / record, buffers rx ${show(h.rx_buffer)} / tx ${show(h.tx_buffer)}`));
    nodes.push(table(
      ["Class", "Name", "Registers", ["Words", "certain extent (max if the size is uncertain)"], ["Layout", "members × entries × member size (words)"],
        "ID map", "Coil base", "Flags", ""],
      dir.records.map((r) => {
        const b = el("button", "Browse", "mini-btn needs-conn");
        b.type = "button";
        b.title = `Read ${r.class} in the raw register browser (read-only)`;
        b.addEventListener("click", () => browseTo(r.class_id));
        return {
          cells: [{ v: r.class, cls: "mono" }, r.name, { v: `${r.address}-${r.end}`, cls: "mono" },
            r.exact ? r.words : `${r.words} (≤ ${r.words_readable ?? r.words_max})`, layoutText(r.members, r.entries, r.member_size),
            { v: r.id_map, cls: "mono" }, { v: r.coil_base, cls: "mono" }, { v: r.flags, cls: "mono" }, b],
        };
      }),
    ));
  }
  box.replaceChildren(...nodes);
  $("sumIdentity").textContent = [id.model, id.serial_number && `S/N ${id.serial_number}`,
    id.firmware && `fw ${id.firmware}`, mode && mode.name].filter(Boolean).join(" · ");
}

function renderParams() {
  const box = $("infoParams");
  const p = info && info.parameters;
  if (!p) {
    box.replaceChildren(placeholder(info ? "Parameters not readable (see errors above)." : "Press Refresh to read the unit."));
    $("sumParams").textContent = "";
    return;
  }
  // The Edit column comes from params.js (the only part of this view that writes, after a confirmation).
  const editCell = (q) => (typeof paramEditButton === "function" ? paramEditButton(q) : "");
  const rows = p.named.map((q) => ({
    title: `${q.encoding}\n${q.evidence}` + (q.model_note ? `\n${q.model_note}` : ""),
    cls: q.found ? (q.listed === false ? "unconfirmed-row" : "") : "missing",
    cells: [`P${q.no}`, q.name,
      { v: q.found ? (q.unit === "mm" && isNum(q.value) ? q.value.toFixed(2) : q.text) : "not found on this EC", cls: "val" },
      document.createTextNode(q.unit || ""), { v: q.found ? `${q.key} @ ${q.address}` : q.key, cls: "mono", title: q.encoding },
      { v: q.raw_hex, cls: "mono" }, editCell(q)],
  }));
  const model = p.model;
  const nodes = [
    table(["No.", "Name", "Value", "Unit", ["Register", "class.item @ address (found by item ID through the class's ID map)"], "Raw",
      ["Edit", "Writes and commits the parameter (asks first); it applies after a software reset"]], rows),
    note(p.note || ""),
  ];
  if (model) nodes.push(note(`Parameter list for this unit: ${model.note}.` + (p.write_note ? ` Edit: ${p.write_note}` : "")));
  if (p.not_located && p.not_located.length) {
    nodes.push(note("Not located: " + p.not_located.map((q) => `P${q.no} ${q.name}` +
      (q.listed && q.iaos_text ? ` - IA-OS shows "${q.iaos_text}" (read-only display, not read from the EC)` : "") +
      ` (${q.note})`).join("; ") + "."));
  }
  nodes.push(heading("Raw items", "every member of the parameter classes by item ID; hex, unsigned, signed and ×0.01"));
  let items = 0;
  for (const c of p.classes || []) {
    items += c.items.length;
    const d = el("details", undefined, "sub");
    const s = el("summary");
    s.append(el("b", c.class), document.createTextNode(` ${c.name} · ${c.items.length} items @ ${c.address}` +
      (c.id_map ? ` (ID map ${c.id_map})` : "")));
    d.append(s, table(
      ["Key", "ID", "Address", "Hex", "Unsigned", "Signed", ["×0.01", "signed × 0.01: the EC's usual mm / mm/s / G unit (a convenience)"], "Param", "Note"],
      c.items.map((it) => ({
        cls: it.params && it.params.length ? "param-row" : "",
        cells: [{ v: it.key, cls: "mono" }, show(it.id), { v: it.address, cls: "mono" }, { v: it.hex, cls: "mono" },
          it.unsigned, it.signed, it.scaled, (it.params || []).join(" "), it.note || ""],
      })),
    ));
    nodes.push(d);
  }
  box.replaceChildren(...nodes);
  const found = p.named.filter((q) => q.found).length;
  $("sumParams").textContent = `${found} / ${p.named.length} named found · ${(p.classes || []).length} classes, ${items} raw items`;
}

function renderAlarms() {
  const box = $("infoAlarms");
  const a = info && info.alarms;
  const nodes = [el("div", undefined, "alarm-now idle")];
  nodes[0].id = "alarmNow";
  if (!a) {
    nodes.push(placeholder(info ? "Alarm history not readable (see errors above)." : "Press Refresh to read the alarm history."));
  } else {
    nodes.push(heading("History", `${a.address}, ${a.slots} slots: ${a.alarm_count} alarms, ${a.power_up_markers} power-up markers, ` +
      `${a.empty_slots} empty`));
    nodes.push(a.history.length ? table(
      [["Slot", "index in the history block (0 is the newest)"], "Code", "Name",
        ["Level", "alarm level: message (warning, the axis keeps running) / operation cancelled / cold start"],
        "Detail", "Address",
        ["Time after power-on", "ms since the EC was powered on (the EC has no clock)"],
        ["Power-ups since", "power-up markers at lower (newer) indices"]],
      a.history.map((h) => [h.index, { v: h.hex, cls: "mono" }, h.name, show(h.level),
        { v: hexN(h.detail), cls: "mono" }, { v: hexN(h.address) + (h.detail_text ? ` (${h.detail_text})` : ""), cls: "mono" },
        show(h.time_after_power_on || h.time_raw), h.newer_power_ups]),
    ) : placeholder("No alarms in the history."));
    nodes.push(el("ul", undefined, "small notes"));
    nodes[nodes.length - 1].append(...Object.values(a.notes || {}).map((n) => el("li", n)));
  }
  box.replaceChildren(...nodes);
  renderAlarmNow();
  const cur = (liveData && liveData.live && liveData.live.alarm) || (a && a.current);
  $("sumAlarms").textContent = [cur ? (cur.code ? `current ${cur.hex} ${cur.name}` : "no current alarm") : "",
    a ? `${a.alarm_count} in history` : ""].filter(Boolean).join(" · ");
}

function renderMaint() {
  const box = $("infoMaint");
  const m = info && info.maintenance;
  if (!m) {
    box.replaceChildren(placeholder(info ? "Maintenance counters not readable (see errors above)." : "Press Refresh to read the unit."));
    $("sumMaint").textContent = "";
    return;
  }
  const tiles = el("div", undefined, "tiles");
  for (const [label, value, unit, small] of [["Total moves · TLMC", m.total_moves, "", "move starts, retargets included"],
    ["Total distance · ODOM", m.distance_m, "m", ""]]) {
    const t = el("div", undefined, "card stat mini");
    const strong = el("strong", show(value));
    strong.append(el("span", unit));
    t.append(el("div", label, "label"), strong, el("div", small, "small"));
    tiles.append(t);
  }
  box.replaceChildren(tiles, table(
    ["ID", "Symbol", "Meaning", "Address", "Hex", "Value"],
    m.items.map((it) => ({
      cls: it.confirmed ? "" : "unconfirmed-row",
      title: it.confirmed ? "VERIFIED-LIVE" : "expected symbol (by item order), unconfirmed",
      cells: [it.id, it.symbol + (it.confirmed ? "" : "?"),
        it.meaning + (it.hi16 !== undefined ? ` (high ${it.hi16}, low ${it.lo16})` : ""),
        { v: it.address, cls: "mono" }, { v: it.hex, cls: "mono" }, it.unset ? "not set" : it.value],
    })),
  ), note(m.note || ""));
  $("sumMaint").textContent = `${show(m.total_moves)} moves · ${show(m.distance_m)} m`;
}

function renderPositions() {
  const box = $("infoPositions");
  const p = info && info.positions;
  if (!p) {
    box.replaceChildren(placeholder(info ? "Position data not readable (see errors above)." : "Press Refresh to read the unit."));
    $("sumPositions").textContent = "";
    return;
  }
  const nodes = [heading("Position table rows 0-2", `${p.mode || ""}; non-volatile, read-only here`)];
  nodes.push(table(
    ["No.", "Role", "Position", "Band", "Speed", "Accel", "Decel", "Zone + / −", "Push", "CTLF", "Checksum", "Address", "Raw words"],
    p.rows.map((r) => ({
      title: `row ${r.n}: ${r.raw}`,
      cells: [r.n, r.role, mmText(r.position_mm), mmText(r.band_mm), `${fix2(r.speed_mm_s)} mm/s`, `${fix2(r.accel_g)} G`,
        `${fix2(r.decel_g)} G`, `${fix2(r.zone_plus_mm)} / ${fix2(r.zone_minus_mm)}`,
        r.push ? `${r.push_pct} % ${r.push_direction}, end ${mmText(r.push_end_mm)}` : "no",
        { v: `${r.control_flags}${r.ctlf_on && r.ctlf_on.length ? " " + r.ctlf_on.join(" ") : ""}`, cls: "mono" },
        { v: `${show(r.checksum)} ${r.checksum_ok ? "ok" : "BAD"}`, cls: r.checksum_ok ? "mono" : "mono bad" },
        { v: r.address, cls: "mono" }, { v: r.raw, cls: "mono raw" }],
    })),
  ));
  nodes.push(heading("Simple-data % rows (260-263)", "the speed / accel / decel settings in %; never written by this driver"));
  nodes.push(table(
    ["No.", "Role", "Position", "Band", "Speed %", "Accel %", "Decel %", "Push power", "CTLF", "Checksum", "Address", "Raw words"],
    p.simple_data_rows.map((r) => [r.n, r.role, mmText(r.position_mm), mmText(r.band_mm), r.speed_pct, r.accel_pct,
      r.decel_pct, r.push_power, { v: r.control_flags, cls: "mono" }, r.checksum_ok ? "ok" : "BAD",
      { v: r.address, cls: "mono" }, { v: r.raw, cls: "mono raw" }]),
  ));
  if (p.simple_data_note) nodes.push(note(p.simple_data_note));
  const dm = p.direct_move;
  if (dm) {
    nodes.push(heading("Last direct-move command (0x9900)", dm.note));
    nodes.push(kvTable([
      ["Target", mmText(dm.target_mm)], ["Band", mmText(dm.band_mm)], ["Speed", `${fix2(dm.speed_mm_s)} mm/s`],
      ["Accel / decel", isNum(dm.accel_g) ? `${fix2(dm.accel_g)} G` : "—"],
      ["Push power", isNum(dm.push_power) ? `${dm.push_power} (${dm.push_pct} %)` : "—"],
      ["CTLF", dm.control_flags ? `${dm.control_flags} ${(dm.ctlf_on || []).join(" ")}` : "—"],
      ["DRG1 mirror (0x9909)", dm.drg1_mirror], ["Raw", dm.raw],
    ]));
  }
  const ap = p.active_profile;
  if (ap) {
    nodes.push(heading("Active move profile (0x0F00)", ap.note));
    nodes.push(kvTable([
      ["Target", mmText(ap.target_mm)], ["Speed", isNum(ap.speed_mm_s) ? `${fix2(ap.speed_mm_s)} mm/s` : "—"],
      ["Accel / decel", isNum(ap.accel_g) ? `${fix2(ap.accel_g)} / ${fix2(ap.decel_g)} G` : "—"],
      ["Band", mmText(ap.band_mm)], ["SPOW / DPOW", `${show(ap.spow)} / ${show(ap.dpow)}`],
      ["Push power", show(ap.push_power)], ["CTLF", show(ap.control_flags)], ["Raw", ap.raw],
    ]));
  }
  box.replaceChildren(...nodes);
  $("sumPositions").textContent = `${p.mode || ""} · rows 0-2: ${p.rows.map((r) => fix2(r.position_mm)).join(" / ")} mm`;
}

function renderBrowserChoices() {
  const select = $("browseClass");
  const records = info && info.identity && info.identity.directory ? info.identity.directory.records : [];
  const keep = select.value;
  if (!records.length) {
    select.replaceChildren(el("option", info ? "Directory not readable" : "Refresh first"));
    select.options[0].value = "";
    return;
  }
  select.replaceChildren(...records.map((r) => {
    const readable = r.words_readable ?? r.words;
    const size = r.exact ? `${r.words} words` : `${r.words}-${readable} words (size uncertain)`;
    const o = el("option", `${r.class} ${r.name} · ${r.address}-${r.end} · ${size}`);
    o.value = String(r.class_id);
    o.dataset.words = String(readable); // the browser may read the uncertain tail too (marked)
    return o;
  }));
  if ([...select.options].some((o) => o.value === keep)) select.value = keep;
  else {
    const params = [...select.options].find((o) => o.value === String(0x0007));
    if (params) select.value = params.value;
    resetBrowseCount();
  }
}

function renderInfo() {
  const errs = $("infoErrors");
  const list = info ? info.errors || [] : [];
  errs.replaceChildren(...list.map((e) => el("li", e)));
  errs.classList.toggle("hidden", !list.length);
  if (info) {
    const meta = info.meta || {};
    const gui = meta.gui || {};
    const when = meta.taken_at ? new Date(meta.taken_at).toLocaleTimeString() : "?";
    $("infoStamp").textContent = `Read at ${when} in ${show(gui.wall_s ?? meta.duration_s)} s` +
      (gui.bus_lock ? ` (bus lock per request; STOP ends a read)` : "") + " · " +
      `function codes ${(meta.function_codes || []).join(" + ")} only · iai_actuator_driver ${show(meta.driver_version)} · ${show(meta.schema)}` +
      (list.length ? ` · ${list.length} error(s)` : "");
  } else {
    $("infoStamp").textContent = last && last.connected
      ? "Not read yet for this connection: press Refresh. The live sections already follow the status poll."
      : "Not connected.";
  }
  $("infoSave").disabled = !info;
  renderIdentity();
  renderParams();
  renderAlarms();
  renderMaint();
  renderPositions();
  renderBrowserChoices();
  renderLive();
}

// ------------------------------------------------------------------ refresh / live loop / connection changes

async function refreshInfo() {
  if (refreshing) return;
  if (!(last && last.connected)) return toast("Not connected.");
  syncConnection();
  refreshing = true;
  const btn = $("infoRefresh");
  btn.textContent = "Reading...";
  btn.classList.add("busy-btn");
  try {
    const r = await api("/api/unit_info");
    if (connKey() === infoConnKey) {
      info = r.info;
      infoVersion += 1;
      renderInfo();
    }
  } catch (e) {
    toast("Unit info: " + e.message);
  } finally {
    refreshing = false;
    btn.textContent = "Refresh";
    btn.classList.remove("busy-btn");
  }
}

function connKey() {
  if (!(last && last.connected && last.connection)) return "";
  const c = last.connection;
  return [c.port, c.slave, c.simulate, c.sim_profile, c.driver].join("|");
}

function infoVisible() {
  return !$("viewInfo").classList.contains("hidden");
}

function syncConnection() {
  const key = connKey();
  if (key === infoConnKey) return false;
  infoConnKey = key;
  info = null;
  infoVersion += 1;
  liveData = null;
  browseBlock = null;
  $("browseTable").replaceChildren();
  for (const id of ["infoIo", "infoStatus", "infoMonitor"]) {
    $(id).dataset.built = "";
    $(id).replaceChildren();
  }
  renderInfo();
  if (key && infoVisible()) refreshInfo(); // first look at a new connection
  return true;
}

async function liveTick() {
  syncConnection();
  if (!infoVisible() || !infoConnKey || liveBusy) return;
  liveBusy = true;
  try {
    const r = await api("/api/unit_live");
    if (connKey() === infoConnKey) {
      liveData = r;
      renderLive();
    }
  } catch (e) {
    // The status badge in the header already reports a lost server / link.
  } finally {
    liveBusy = false;
  }
}

// ------------------------------------------------------------------ raw register browser (read-only)

function selectedWords() {
  const o = $("browseClass").selectedOptions[0];
  return o && o.dataset.words ? Number(o.dataset.words) : null;
}

function resetBrowseCount() {
  const words = selectedWords();
  $("browseOffset").value = "0";
  if (words) $("browseCount").value = String(Math.min(words, 256));
}

async function readBrowse() {
  const cls = $("browseClass").value;
  if (!cls) return toast("Press Refresh first: the block list comes from the EC's own directory.");
  const offset = Number($("browseOffset").value || 0);
  let count = Number($("browseCount").value || 0);
  if (!Number.isInteger(offset) || offset < 0 || !Number.isInteger(count) || count < 1 || count > 512) {
    return toast("Offset must be ≥ 0 and count 1-512 (whole numbers).");
  }
  const words = selectedWords();
  if (words !== null) {
    if (offset >= words) return toast(`This block has ${words} registers: offset ${offset} is outside it.`);
    count = Math.min(count, words - offset); // the last page of a block may be shorter
  }
  const q = `class=0x${Number(cls).toString(16).toUpperCase().padStart(4, "0")}&offset=${offset}&count=${count}`;
  try {
    browseBlock = (await api("/api/read_block?" + q)).block;
    browseBlock.readAt = new Date().toLocaleTimeString();
    renderBrowse();
  } catch (e) {
    toast("Read registers: " + e.message);
  }
}

function wordText(w, view) {
  if (view === "dec") return String(w);
  if (view === "signed") return String(w >= 0x8000 ? w - 0x10000 : w);
  if (view === "ascii") {
    const ch = (c) => (c >= 0x20 && c < 0x7f ? String.fromCharCode(c) : "·");
    return ch(w >> 8) + ch(w & 0xff);
  }
  return w.toString(16).toUpperCase().padStart(4, "0");
}

function renderBrowse() {
  const b = browseBlock;
  const t = $("browseTable");
  if (!b) return t.replaceChildren();
  const view = $("browseView").value;
  const end = b.address + b.count - 1;
  const blockEnd = b.block_address + b.block_words - 1;
  const readable = b.words_readable ?? b.block_words;
  const tail = !b.exact && readable > b.block_words;
  $("browseInfo").textContent = `${b.class} ${b.name}: block ${hexN(b.block_address)}-${hexN(blockEnd)} (${b.block_words} words` +
    `${tail ? `; size uncertain: up to ${readable} (to ${hexN(b.block_address + readable - 1)}), the words past ` +
      `${hexN(blockEnd)} are shown in italics, hatched` : ""}; ${layoutText(b.members, b.entries, b.member_size, true)}). ` +
    `Read ${hexN(b.address)}-${hexN(end)} (${b.count} words, offset ${b.offset}) at ${b.readAt}. Read-only (FC03).`;
  const head = el("tr");
  head.append(el("th", "Address"));
  for (let i = 0; i < 8; i++) head.append(el("th", `+${i}`));
  const thead = el("thead");
  thead.append(head);
  const body = el("tbody");
  for (let i = 0; i < b.words.length; i += 8) {
    const tr = el("tr");
    tr.append(el("th", hexN(b.address + i), "mono"));
    for (let j = i; j < Math.min(i + 8, b.words.length); j++) {
      const w = b.words[j];
      const uncertain = !b.exact && b.offset + j >= b.block_words;
      const td = el("td", wordText(w, view), "mono" + (w === 0 ? " zero" : "") + (uncertain ? " uncertain" : ""));
      td.title = `${hexN(b.address + j)} (offset ${b.offset + j}): 0x${wordText(w, "hex")} = ${w} unsigned, ${wordText(w, "signed")} signed, "${wordText(w, "ascii")}"` +
        (uncertain ? " - past the block's certain size (size uncertain)" : "");
      tr.append(td);
    }
    body.append(tr);
  }
  t.replaceChildren(thead, body);
}

function page(step) {
  const words = selectedWords();
  const count = Number($("browseCount").value || 0);
  const offset = Number($("browseOffset").value || 0);
  if (!words || !(count > 0) || !Number.isInteger(offset)) return;
  if ((step < 0 && offset === 0) || (step > 0 && offset + count >= words)) return; // no page there
  $("browseOffset").value = String(Math.max(0, Math.min(offset + step * count, words - 1)));
  readBrowse();
}

function browseTo(classId) {
  $("secBrowser").open = true;
  $("browseClass").value = String(classId);
  resetBrowseCount();
  readBrowse();
  $("secBrowser").scrollIntoView({ behavior: "smooth", block: "start" });
}

// ------------------------------------------------------------------ view switching, folding, save

function showView(id) {
  for (const b of document.querySelectorAll(".views [role=tab]")) {
    const on = b.dataset.view === id;
    b.classList.toggle("active", on);
    b.setAttribute("aria-selected", String(on));
  }
  $("viewOperate").classList.toggle("hidden", id !== "viewOperate");
  $("viewInfo").classList.toggle("hidden", id !== "viewInfo");
  try {
    localStorage.setItem("ecgui.view", id);
  } catch {
    /* per-viewer convenience only */
  }
  if (id === "viewInfo") {
    syncConnection();
    if (!info && infoConnKey) refreshInfo();
    liveTick();
  }
}

function saveFolds() {
  try {
    localStorage.setItem("ecgui.folds", JSON.stringify(Object.fromEntries(INFO_SECTIONS.map((id) => [id, $(id).open]))));
  } catch {
    /* per-viewer convenience only */
  }
}

function restoreFolds() {
  try {
    const saved = JSON.parse(localStorage.getItem("ecgui.folds") || "{}");
    for (const id of INFO_SECTIONS) if (typeof saved[id] === "boolean") $(id).open = saved[id];
  } catch {
    /* ignore */
  }
}

function saveJson() {
  if (!info) return;
  const id = info.identity || {};
  const stamp = (info.meta && info.meta.taken_at ? info.meta.taken_at : new Date().toISOString()).slice(0, 19);
  const name = ["unit-info", id.model, id.serial_number, stamp].filter(Boolean).join("_").replace(/[^\w.-]+/g, "-");
  const blob = new Blob([JSON.stringify(info, null, 2)], { type: "application/json" });
  const a = el("a");
  a.href = URL.createObjectURL(blob);
  a.download = name + ".json";
  document.body.append(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 2000);
}

for (const b of document.querySelectorAll(".views [role=tab]")) b.addEventListener("click", () => showView(b.dataset.view));
$("infoRefresh").addEventListener("click", refreshInfo);
$("infoExpand").addEventListener("click", () => {
  for (const id of INFO_SECTIONS) $(id).open = true;
  saveFolds();
});
$("infoCollapse").addEventListener("click", () => {
  for (const id of INFO_SECTIONS) $(id).open = false;
  saveFolds();
});
for (const id of INFO_SECTIONS) $(id).addEventListener("toggle", saveFolds);
$("infoSave").addEventListener("click", saveJson);
$("browseForm").addEventListener("submit", (e) => {
  e.preventDefault();
  readBrowse();
});
$("browseClass").addEventListener("change", resetBrowseCount);
$("browseView").addEventListener("change", renderBrowse);
$("browsePrev").addEventListener("click", () => page(-1));
$("browseNext").addEventListener("click", () => page(1));

restoreFolds();
renderInfo();
setInterval(liveTick, INFO_LIVE_MS);
try {
  if (localStorage.getItem("ecgui.view") === "viewInfo") showView("viewInfo");
} catch {
  /* ignore */
}
