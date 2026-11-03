"use strict";
// EC test GUI client. Polls /api/status, renders tiles and the event log, sends commands.
// Every limit checked here is checked again by the server.

const TOKEN = window.ECToken.get(); // the access key from the launch address (token.js)
const $ = (id) => document.getElementById(id);
const POLL_MS = 250;
const PUSH_FORCE = [20, 70]; // % (the GUI's range; the max is lowered to the EC's MXPP, item 7.9, per connection)
const pushRange = () => [
  typeof limits.min_push_pct === "number" ? limits.min_push_pct : PUSH_FORCE[0],
  typeof limits.max_push_pct === "number" ? limits.max_push_pct : PUSH_FORCE[1],
];
// The server reports a max push of 0 % for a unit that cannot push (MXPP = 0, or no push support).
const cannotPush = () => typeof limits.max_push_pct === "number" && limits.max_push_pct <= 0;
const CANNOT_PUSH_TEXT = "This unit cannot push (its push cap is 0 %): push is disabled.";

// Indicator lamps: [status key, label, colour when lit, source shown as a tooltip].
const LAMPS = [
  ["servo_on", "Servo ON", "ok", "DSS1 b12 SV"],
  ["homed", "Homed", "ok", "DSS1 b4 HEND"],
  ["in_position", "In position", "ok", "DSS1 b3 PEND"],
  ["moving", "Moving", "busy", "Driver: MOVE, homing, or servo on without PEND"],
  ["alarm", "Alarm", "alert", "A fault: DSS1 b10 ALMH, or an alarm of level 'operation cancelled' / 'cold start' (blocks motion)"],
  ["warning", "Warning", "warn", "A message-level alarm (0x011-0x07F, e.g. a maintenance warning): the axis keeps running"],
  ["emergency_stop", "Emergency stop", "alert", "DSS1 b15 EMGS"],
  ["modbus_control", "Modbus control", "ok", "DSSE b8 PMSS (off: PIO has control)"],
  ["safety_speed", "Safety speed", "warn", "DSS1 b14 SFTY: the EC caps speed at ~100 mm/s"],
  ["paused", "Paused", "warn", "DSS1 b5 STP (pause input on)"],
  ["homing", "Homing", "busy", "DSSE b11 GHMS"],
  ["target_reached", "Target reached", "ok", "DSS1 b0 MEND: the last move ended at its target"],
  ["pushing", "Pushing", "busy", "DSSE b10: push in progress or holding after contact"],
  ["push_contact", "Push contact", "ok", "PEND + DSSE b0: holding against the work"],
  ["push_missed", "Push missed", "warn", "DSS1 b11 PSFL: no contact over the band"],
];
// Position markers come from ZONS 0x9013 (same bits on every EC tested). DOPM is shown raw only:
// its layout differs per model (b2 = Middle on the 3-position slider, *ALM on the 2-position gripper).
// They follow the position, also while moving: lit within the LS range (EC parameter 2, 0.10 mm on
// both units tested) of the stored position; for a push row, of its push end.
const MARKERS = [
  ["at_backward_end", "Backward end", "ok", "ZONS b12 LS0 (within 0.10 mm of position 0)"],
  ["at_middle", "Middle", "ok", "ZONS b14 LS2 (within 0.10 mm of position 2; lit on 2-position units too)"],
  ["at_forward_end", "Forward end", "ok", "ZONS b13 LS1 (within 0.10 mm of position 1; a push row: its push end)"],
];
const RAW_SHOWN = ["DSS1", "DSSE", "ZONS", "DOPM", "ALMC"];

// Stroke null = not known yet (the EC's own soft limits are read at connect).
let limits = { stroke_min_mm: null, stroke_max_mm: null, max_speed_mm_s: 100, max_accel_g: 0.3, max_speed_source: "GUI cap" };
// The axis's position unit from the server ("deg" on a rotary axis): only the label differs, the commands are the same.
const U = () => (limits && limits.units) || "mm";
const strokeKnown = () => typeof limits.stroke_min_mm === "number" && typeof limits.stroke_max_mm === "number";
let last = null; // last /api/status payload
let wasConnected = false;
let lastEventKey = "";

async function api(path, data) {
  const r = await fetch(path, {
    method: data === undefined ? "GET" : "POST",
    headers: { "X-EC-Token": TOKEN, "Content-Type": "application/json" },
    body: data === undefined ? undefined : JSON.stringify(data),
  });
  let v;
  try {
    v = await r.json();
  } catch {
    throw Error(`Server returned ${r.status} without JSON`);
  }
  if (!r.ok || v.ok === false) throw Error(v.error || `Request failed (${r.status})`);
  return v;
}

let toastTimer = null;
function toast(message) {
  $("toast").textContent = message;
  $("toast").classList.remove("hidden");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => $("toast").classList.add("hidden"), 8000);
}

function el(tag, text, cls) {
  const e = document.createElement(tag);
  if (text !== undefined) e.textContent = text;
  if (cls) e.className = cls;
  return e;
}

// ------------------------------------------------------------------ STOP

async function stopNow(takeControl = false) {
  const b = $("stopBtn");
  b.classList.add("pressed");
  try {
    await api("/api/stop", takeControl ? { take_control: true } : {});
  } catch (e) {
    toast("STOP: " + e.message);
  } finally {
    setTimeout(() => b.classList.remove("pressed"), 150);
  }
}
$("stopBtn").addEventListener("click", () => stopNow());

function typingInText(target) {
  return target && ((target.tagName === "INPUT" && target.type === "text") || target.isContentEditable);
}
// Esc always stops. Space stops unless typing in a text field. Capture phase + preventDefault so
// Space never "clicks" whichever button happens to have focus.
window.addEventListener(
  "keydown",
  (e) => {
    const space = e.code === "Space" || e.key === " ";
    if (e.key === "Escape" || (space && !typingInText(e.target))) {
      e.preventDefault();
      e.stopPropagation();
      if (!e.repeat) stopNow();
    }
  },
  true,
);
window.addEventListener(
  "keyup",
  (e) => {
    if ((e.code === "Space" || e.key === " ") && !typingInText(e.target)) e.preventDefault();
  },
  true,
);

// ------------------------------------------------------------------ commands

async function command(path, data) {
  hideMoveError();
  try {
    await api(path, data);
  } catch (e) {
    toast(e.message);
  }
}

function showMoveError(message) {
  $("moveError").textContent = message;
  $("moveError").classList.remove("hidden");
}
function hideMoveError() {
  $("moveError").classList.add("hidden");
  for (const id of ["target", "speed", "accel"]) $(id).classList.remove("invalid");
}

function readNumber(id, name) {
  const raw = $(id).value.trim();
  const v = Number(raw);
  if (raw === "" || !Number.isFinite(v)) {
    $(id).classList.add("invalid");
    throw Error(`${name} must be a number.`);
  }
  return v;
}

function speedLimitText() {
  const source = limits.max_speed_source && limits.max_speed_source !== "GUI cap" ? ` (${limits.max_speed_source})` : "";
  return `Speed must be > 0 and ≤ ${limits.max_speed_mm_s} ${U()}/s${source}.`;
}

function readProfile() {
  const speed = readNumber("speed", "Speed");
  if (!(speed > 0 && speed <= limits.max_speed_mm_s)) {
    $("speed").classList.add("invalid");
    throw Error(speedLimitText());
  }
  const accel = readNumber("accel", "Accel");
  if (!(accel > 0 && accel <= limits.max_accel_g)) {
    $("accel").classList.add("invalid");
    throw Error(`Accel must be > 0 and ≤ ${limits.max_accel_g} G.`);
  }
  return { speed, accel };
}

function checkTarget(position, what) {
  if (!strokeKnown()) return; // not known here yet: the server checks against the EC's limits
  if (!(position >= limits.stroke_min_mm && position <= limits.stroke_max_mm))
    throw Error(`${what} ${position.toFixed(2)} ${U()} is outside the stroke ${limits.stroke_min_mm}–${limits.stroke_max_mm} ${U()}.`);
}

async function moveTo(position) {
  hideMoveError();
  let profile;
  try {
    checkTarget(position, "Target");
    profile = readProfile();
  } catch (e) {
    return showMoveError(e.message);
  }
  try {
    await api("/api/move", { position_mm: position, speed_mm_s: profile.speed, accel_g: profile.accel });
  } catch (e) {
    showMoveError(e.message);
  }
}

$("moveForm").addEventListener("submit", (e) => {
  e.preventDefault();
  hideMoveError();
  let target;
  try {
    target = readNumber("target", "Target");
  } catch (err) {
    return showMoveError(err.message);
  }
  if (strokeKnown() && !(target >= limits.stroke_min_mm && target <= limits.stroke_max_mm)) $("target").classList.add("invalid");
  moveTo(target);
});

for (const b of document.querySelectorAll("[data-jog]")) {
  b.addEventListener("click", async () => {
    hideMoveError();
    const delta = Number(b.dataset.jog);
    let profile;
    try {
      profile = readProfile();
      const pos = last?.status?.position_mm;
      if (typeof pos === "number") checkTarget(pos + delta, `Jog ${delta > 0 ? "+" : ""}${delta} ${U()} →`);
    } catch (e) {
      return showMoveError(e.message);
    }
    try {
      await api("/api/jog", { delta_mm: delta, speed_mm_s: profile.speed, accel_g: profile.accel });
    } catch (e) {
      showMoveError(e.message);
    }
  });
}

$("servoOnBtn").addEventListener("click", () => command("/api/servo", { on: true }));
$("servoOffBtn").addEventListener("click", () => command("/api/servo", { on: false }));
$("homeBtn").addEventListener("click", () => {
  if (confirm("Home return: the axis moves at ~30 mm/s to its mechanical end, then back to 0.00 mm. Start?"))
    command("/api/home", {});
});
$("resetBtn").addEventListener("click", () => command("/api/reset_alarm", {}));

// ------------------------------------------------------------------ configuration (software reset, baud)

$("swResetBtn").addEventListener("click", async () => {
  if (
    !confirm(
      "SOFTWARE RESET\n\nThe servo is switched OFF, then the EC restarts its firmware (0x45 cmd 5). " +
        "Committed parameter changes take effect. Afterwards PIO has control again and safety speed is off.\n\n" +
        "Incremental units (no WA option) LOSE THEIR HOME: run Home before the next move. An absolute unit whose home " +
        "parameters (P3 / P4) changed comes up with ABS error 0x0EE (Alarm reset + Home).\n\nReset now?",
    )
  )
    return;
  const b = $("swResetBtn");
  b.textContent = "Resetting...";
  try {
    const r = (await api("/api/software_reset", {})).result;
    toast(
      `Software reset done: PMSS ${r.modbus_enabled ? 1 : 0}, SFTY ${r.safety_speed ? 1 : 0}, servo ${r.servo_on ? "ON" : "OFF"}` +
        `, HEND ${r.homed ? 1 : 0}` + (r.needs_home ? " - NEEDS HOME before the next move" : ""),
    );
  } catch (e) {
    toast("Software reset: " + e.message);
  } finally {
    b.textContent = "Software reset";
  }
});

$("baudForm").addEventListener("submit", async (e) => {
  e.preventDefault();
  const rate = Number($("newBaud").value);
  const method = $("baudMethod").value;
  const now = last?.connection?.line_baud;
  if (
    !confirm(
      `Change the EC's baud rate${typeof now === "number" ? ` from ${now}` : ""} to ${rate} (${method})?\n\n` +
        "Broadcast baud change: EVERY EC on this RS-485 line switches. Volatile: the EC answers at 38400 again " +
        "after a power-up (the driver auto-detects at connect).",
    )
  )
    return;
  try {
    const r = (await api("/api/baud", { rate, method })).result;
    toast(`Baud ${r.previous} -> ${r.baudrate} (${r.verified ? "verified" : "found by probing"})`);
  } catch (err) {
    toast("Baud change: " + err.message);
  }
});

function renderReset(s, st) {
  const r = s.connected ? s.swreset : null;
  const line = s.connected && typeof s.connection.line_baud === "number" ? `Line ${s.connection.line_baud} baud. ` : "";
  if (!r) {
    $("resetInfo").textContent = line;
    $("resetInfo").classList.remove("error");
    return;
  }
  const needsHome = r.needs_home && st.homed !== true;
  $("resetInfo").textContent =
    line +
    `Software reset at ${r.at}: PMSS ${r.modbus_enabled ? 1 : 0}, SFTY ${r.safety_speed ? 1 : 0}, servo ` +
    `${r.servo_on ? "ON" : "OFF"}, HEND ${r.homed ? 1 : 0}` +
    (needsHome ? " - NEEDS HOME: run Home before the next move." : r.needs_home ? " - home since re-established." : ".");
  $("resetInfo").classList.toggle("error", needsHome);
}

// ------------------------------------------------------------------ push

function showPushError(message) {
  $("pushError").textContent = message;
  $("pushError").classList.remove("hidden");
}

$("pushForm").addEventListener("submit", async (e) => {
  e.preventDefault();
  $("pushError").classList.add("hidden");
  for (const id of ["pushTarget", "pushBand", "pushForce", "pushSpeed", "accel"]) $(id).classList.remove("invalid");
  let body;
  try {
    if (cannotPush()) throw Error(CANNOT_PUSH_TEXT);
    const target = readNumber("pushTarget", "Approach position");
    const band = readNumber("pushBand", "Band");
    const force = readNumber("pushForce", "Force");
    const speed = readNumber("pushSpeed", "Speed");
    const accel = readNumber("accel", "Accel");
    const direction = Number($("pushDir").value);
    checkTarget(target, "Approach position");
    if (!(band > 0)) throw Error(`Band must be > 0 ${U()}.`);
    checkTarget(target + direction * band, "Band end");
    const [pushMin, pushMax] = pushRange();
    if (!(force >= pushMin && force <= pushMax)) {
      $("pushForce").classList.add("invalid");
      throw Error(`Force must be ${pushMin}–${pushMax} %.`);
    }
    if (!(speed > 0 && speed <= limits.max_speed_mm_s)) throw Error(speedLimitText());
    if (!(accel > 0 && accel <= limits.max_accel_g)) throw Error(`Accel must be > 0 and ≤ ${limits.max_accel_g} G.`);
    body = { target_mm: target, band_mm: band, force_pct: force, direction, speed_mm_s: speed, accel_g: accel };
  } catch (err) {
    return showPushError(err.message);
  }
  try {
    await api("/api/push", body);
  } catch (err) {
    showPushError(err.message);
  }
});

const PUSH_STATE_TEXT = {
  running: "Pushing",
  contact: "CONTACT",
  missed: "MISSED",
  stopped: "Stopped",
  failed: "Failed",
  superseded: "Ended",
};

function renderPush(push) {
  const box = $("pushResult");
  if (!push) {
    box.className = "push-result idle";
    $("pushState").textContent = "No push yet";
    $("pushMsg").textContent = "";
    return;
  }
  box.className = "push-result " + push.state;
  $("pushState").textContent = PUSH_STATE_TEXT[push.state] || push.state;
  const dir = push.direction > 0 ? "away from home" : "toward home";
  $("pushMsg").textContent =
    `${push.message} (approach ${push.target_mm} ${U()}, band ${push.band_mm} ${U()} ${dir} to ${push.end_mm} ${U()}, ${push.force_pct} %)`;
}

// ------------------------------------------------------------------ connect

$("connectForm").addEventListener("submit", async (e) => {
  e.preventDefault();
  const slave = Number($("slave").value);
  if (!Number.isInteger(slave) || slave < 1 || slave > 247) return toast("Slave address must be 1–247.");
  // The stroke fields are an optional override: empty = the EC's own soft limits (read at connect).
  const minText = $("strokeMin").value.trim();
  const maxText = $("strokeMax").value.trim();
  const body = {
    port: $("port").value.trim(),
    baud: Number($("baud").value),
    slave,
    simulate: $("simulate").checked,
    sim_profile: $("simProfile").value,
  };
  if (minText !== "" || maxText !== "") {
    const strokeMin = Number(minText);
    const strokeMax = Number(maxText);
    if (minText === "" || maxText === "" ||
        !(Number.isFinite(strokeMin) && Number.isFinite(strokeMax) && strokeMin >= 0 && strokeMin < strokeMax))
      return toast("Stroke override: leave both fields empty (use the EC's own limits) or give 0 ≤ min < max (mm).");
    body.stroke_min_mm = strokeMin;
    body.stroke_max_mm = strokeMax;
  }
  $("connectBtn").disabled = true;
  try {
    // Identity and the position table are loaded by render() once status shows "connected".
    await api("/api/connect", body);
  } catch (err) {
    toast(err.message);
  } finally {
    $("connectBtn").disabled = !!last?.connected;
  }
});
$("disconnectBtn").addEventListener("click", () => command("/api/disconnect", {}));

function renderIdentity(identity) {
  const body = $("identity");
  body.replaceChildren();
  if (!identity || !Object.keys(identity).length) {
    const tr = el("tr");
    tr.append(el("td", identity === null ? "Identify did not answer (see event log)." : "Not connected.", "small"));
    return body.append(tr);
  }
  for (const [k, v] of Object.entries(identity)) {
    const tr = el("tr");
    tr.append(el("th", k), el("td", typeof v === "object" && v !== null ? JSON.stringify(v) : String(v)));
    body.append(tr);
  }
}

async function loadIdentity() {
  try {
    renderIdentity((await api("/api/identify")).identity);
  } catch (e) {
    renderIdentity(null);
  }
}

// Stored-position move (0x9800 = n): the row's own speed / accel; asks first, twice when above the caps.
async function runStored(row) {
  hideMoveError();
  const flags = typeof row.control_flags === "number" ? row.control_flags : 0;
  const push = (flags & 2) !== 0;
  const accel = Math.max(row.accel_g ?? 0, row.decel_g ?? 0);
  const fast = row.speed_mm_s > limits.max_speed_mm_s || accel > limits.max_accel_g;
  const pushText = push
    ? `, then PUSH ${row.band_mm} ${U()} ${(flags & 4) !== 0 ? "away from" : "toward"} home at ` +
      `${Math.round(((row.push_power || 0) * 100) / 255)} %`
    : "";
  let msg =
    `Run stored position ${row.n} (${row.label}): ${mm(row.position_mm)} ${U()} at its STORED speed ${mm(row.speed_mm_s)} ${U()}/s, ` +
    `accel ${mm(row.accel_g)} / decel ${mm(row.decel_g)} G${pushText}?`;
  if (!confirm(msg)) return;
  if (fast) {
    msg =
      `The row's own speed / accel (${mm(row.speed_mm_s)} ${U()}/s, ${mm(accel)} G) is ABOVE this session's caps ` +
      `(${limits.max_speed_mm_s} ${U()}/s, ${limits.max_accel_g} G). Run it at the row's own values anyway?`;
    if (!confirm(msg)) return;
  }
  try {
    await api("/api/run_stored", { n: row.n, allow_above_caps: fast });
  } catch (e) {
    showMoveError(e.message);
  }
}

async function loadPositions() {
  const box = $("quickMoves");
  const stored = $("storedMoves");
  box.replaceChildren(el("span", "Reading position table...", "small"));
  stored.replaceChildren();
  try {
    const r = await api("/api/positions");
    box.replaceChildren();
    for (const row of r.rows) {
      const pos = row.position_mm;
      const ok = typeof pos === "number";
      const inside = ok && strokeKnown() && pos >= limits.stroke_min_mm && pos <= limits.stroke_max_mm;
      // Not "needs-conn": render() would re-enable it on every poll.
      const b = el("button", `${row.label} (${row.n}) · ${ok ? pos.toFixed(2) + " " + U() : "?"}`, inside ? "needs-conn" : "");
      b.type = "button";
      b.disabled = !inside;
      b.title = !ok ? "Position not readable" : inside ? `Move to ${pos.toFixed(2)} ${U()}` : "Outside the stroke in use";
      if (inside) b.addEventListener("click", () => moveTo(pos));
      box.append(b);
      const speed = typeof row.speed_mm_s === "number" ? ` @ ${mm(row.speed_mm_s)} ${U()}/s` : "";
      const run = el("button", `Run ${row.n} (${row.label})${speed}`, inside ? "needs-conn" : "");
      run.type = "button";
      run.disabled = !inside;
      run.title = inside ? "0x9800 = " + row.n + ": the row's own stored speed / accel (asks first)" : "Outside the stroke in use";
      if (inside) run.addEventListener("click", () => runStored(row));
      stored.append(run);
    }
    const reload = el("button", "Reload", "needs-conn");
    reload.type = "button";
    reload.addEventListener("click", loadPositions);
    box.append(reload);
  } catch (e) {
    box.replaceChildren(el("span", "Could not read the position table: " + e.message, "small"));
  }
}

// ------------------------------------------------------------------ rendering

function yesNo(v, yes = "Yes", no = "No") {
  return v === true ? yes : v === false ? no : "—";
}
function tile(id, text, cls) {
  const t = $(id);
  t.classList.remove("on", "alert", "busy");
  if (cls) t.classList.add(cls);
  t.querySelector("strong").textContent = text;
}

function buildLamps(id, defs) {
  const box = $(id);
  for (const [key, label, kind, source] of defs) {
    const lamp = el("div", undefined, "lamp unknown");
    lamp.dataset.key = key;
    lamp.dataset.kind = kind;
    lamp.title = source;
    lamp.append(el("i"), el("span", label));
    box.append(lamp);
  }
}
buildLamps("lamps", LAMPS);
buildLamps("markers", MARKERS);

function renderLamps(st, connected) {
  // Only the Operate view's lamps: the Unit info view (unitinfo.js) draws its own bit lamps.
  for (const lamp of document.querySelectorAll("#lamps .lamp, #markers .lamp")) {
    const v = connected ? st[lamp.dataset.key] : undefined;
    lamp.className = "lamp " + (v === true ? "on " + lamp.dataset.kind : v === false ? "off" : "unknown");
  }
  const raw = connected && st.raw ? st.raw : {};
  $("rawRegs").textContent = RAW_SHOWN.filter((k) => typeof raw[k] === "number")
    .map((k) => `${k} 0x${raw[k].toString(16).toUpperCase().padStart(4, "0")}`)
    .join(" · ");
}

function mm(v) {
  return typeof v === "number" ? (Math.round(v * 100) / 100).toString() : "?";
}

let strokePrefilled = false;
function renderStroke(s) {
  // Prefill once from a server --stroke override; afterwards the form keeps what the user typed.
  if (!strokePrefilled && s.default_stroke && $("strokeMin").value === "" && $("strokeMax").value === "") {
    [$("strokeMin").value, $("strokeMax").value] = s.default_stroke;
  }
  strokePrefilled = true;
  const info = s.connected ? s.connection.stroke : null;
  const range = (r) => (r ? `${mm(r[0])}–${mm(r[1])} ${U()}` : "none");
  $("strokeInfo").textContent = info
    ? `Stroke in use ${range(info.in_use_mm)} (` +
      (info.configured_mm ? `override ${range(info.configured_mm)}` : "no override: the EC's own") + "). " +
      (info.ec_soft_limits_mm
        ? `EC soft limits read at connect: ${mm(info.ec_soft_limits_mm[0])} / ${mm(info.ec_soft_limits_mm[1])} ${U()} ` +
          `(EC stroke ${range(info.ec_stroke_mm)}).`
        : `EC soft limits: ${info.ec_soft_limits_note}.`) +
      (typeof info.ec_max_speed_mm_s === "number" ? ` EC max speed ${mm(info.ec_max_speed_mm_s)} ${U()}/s.` : "") +
      (info.table_mm ? ` Position table ${info.table_mm[0].toFixed(2)}–${info.table_mm[1].toFixed(2)} ${U()}.` : "")
    : "Leave the stroke empty to use the EC's own soft limits (read at connect); an override can only narrow them.";
  const list = $("strokeWarn");
  const warnings = info ? info.warnings : [];
  if (list.dataset.key !== warnings.join("\n")) {
    list.dataset.key = warnings.join("\n");
    list.replaceChildren(...warnings.map((w) => el("li", w)));
  }
  list.classList.toggle("hidden", !warnings.length);
}

function renderStopWarning(s, st) {
  // Alerts: an unconfirmed STOP, or (with PIO in control) the axis moving again after STOP. A Modbus STOP
  // is verified on Modbus-commanded moves only, not on moves a PLC commanded over PIO.
  $("stopWarnText").textContent = s.stop_alert || "";
  $("stopWarn").classList.toggle("hidden", !s.stop_alert);
  const pio = s.connected && st.modbus_control === false;
  $("pioNote").classList.toggle("hidden", !pio || !!s.pio_hold); // PIO chosen here: the control text says it
}

// ------------------------------------------------------------------ control mode (PC / PIO)

const MODE_CONFIRM = {
  pio:
    "SWITCH TO PIO CONTROL\n\nIn PIO mode the unit follows its PIO inputs (the PLC) and runs its stored position rows at " +
    "their own stored speeds and accelerations, not the limits set on this page. Motion commands from this page are " +
    "refused until you switch back to PC.\n\nA Modbus STOP is untested on a move a PLC commanded over PIO: stop those " +
    "from the PLC or with the hardware stop.\n\nOnly at rest. Switch to PIO now?",
  pc:
    "SWITCH TO PC CONTROL\n\nThe unit takes its commands from this page over Modbus. A PLC that drives it over PIO " +
    "no longer controls it until you switch back to PIO (or disconnect, if PIO had control when you connected).\n\n" +
    "Only at rest. Switch to PC now?",
};

function renderControlMode(s, st) {
  const c = s.connected;
  const pc = st.modbus_control;
  const known = c && (pc === true || pc === false);
  const busy = c && (st.moving !== false || st.homing === true || st.pushing === true);
  const notAtRest = busy || (c && st.paused === true);
  // taking PC control from PIO: a pause at a standstill with the servo off does not block it (a PCON in PIO mode with
  // nothing on its PIO connector pauses at rest); a pause with the servo on is a hold; the driver refuses it while
  // anything moves
  const pcBlocked = pc === false ? busy || (c && st.paused === true && st.servo_on !== false) : notAtRest;
  $("modePcBtn").setAttribute("aria-pressed", known && pc ? "true" : "false");
  $("modePioBtn").setAttribute("aria-pressed", known && !pc ? "true" : "false");
  $("modePcBtn").disabled = !c || pcBlocked;
  $("modePioBtn").disabled = !c || notAtRest;
  let text = "";
  if (c && !known) text = "The unit does not report who has control (PMSS).";
  else if (c && pc) text = "PC has control (PMSS = 1): the unit takes its commands from this page.";
  else if (c && s.pio_hold)
    text =
      "PIO has control (PMSS = 0, chosen here): the unit follows its PIO inputs and runs its stored rows at their own " +
      "speeds. Motion commands from this page are refused until you switch to PC. A Modbus STOP is untested on a move " +
      "a PLC commanded over PIO: use the PLC or the hardware stop for those.";
  else if (c) text = "PIO has control (PMSS = 0, as after power-up). The first command takes PC control at rest, or switch here.";
  if (pc === false ? pcBlocked : notAtRest) text += " The mode is switched only at rest: wait, or press STOP.";
  $("controlInfo").textContent = text;
}

async function chooseControl(mode) {
  const pc = last && last.connected && last.status ? last.status.modbus_control : null;
  if (pc === (mode === "pc")) return; // already in that mode
  if (!confirm(MODE_CONFIRM[mode])) return;
  try {
    await api("/api/control", { mode });
    toast(mode === "pc" ? "PC control: the unit takes its commands from this page." : "PIO control: the unit follows its PIO inputs.");
  } catch (e) {
    toast("Control mode: " + e.message);
  }
}
$("modePcBtn").addEventListener("click", () => chooseControl("pc"));
$("modePioBtn").addEventListener("click", () => chooseControl("pio"));

function render(s) {
  const c = s.connected;
  if (s.limits) limits = s.limits;
  document.querySelectorAll(".u").forEach((e) => {
    if (e.textContent !== U()) e.textContent = U();
  });
  const strokeText = strokeKnown() ? `${limits.stroke_min_mm}–${limits.stroke_max_mm} ${U()}` : "stroke = the EC's own soft limits (read at connect)";
  const speedSource = limits.max_speed_source && limits.max_speed_source !== "GUI cap" ? ` (${limits.max_speed_source})` : "";
  $("limitsInfo").textContent =
    `Limits: ${strokeText}, speed ≤ ${limits.max_speed_mm_s} ${U()}/s${speedSource}, accel ≤ ${limits.max_accel_g} G.` +
    (s.motion_block ? " " + s.motion_block : "");
  renderStroke(s);

  const badge = $("connBadge");
  badge.className = "badge" + (c ? (s.comm_ok ? "" : " bad") : " off");
  badge.textContent = !c
    ? "Disconnected"
    : (s.connection.simulate
        ? "Simulator" + (s.connection.sim_profile ? ` (${s.connection.sim_profile})` : "")
        : `${s.connection.port} · ${s.connection.baud} baud`) +
      ` · slave ${s.connection.slave}` + (s.comm_ok ? "" : " · NO RESPONSE");

  for (const b of document.querySelectorAll(".needs-conn")) b.disabled = !c;
  const noPush = c && cannotPush();
  $("pushBtn").disabled = !c || noPush;
  $("pushUnavailable").classList.toggle("hidden", !noPush);
  for (const id of ["port", "baud", "slave", "strokeMin", "strokeMax", "simulate", "connectBtn"]) $(id).disabled = c;
  $("simProfile").disabled = c || !$("simulate").checked;
  $("disconnectBtn").disabled = !c;
  $("driverInfo").textContent = c
    ? s.connection.driver
    : "Not connected. Connecting only reads from the unit. Close IA-OS (and any other program using the port) first.";

  $("lastError").textContent = s.last_error ? "Last error: " + s.last_error : "";
  $("lastError").classList.toggle("hidden", !s.last_error);

  const st = c ? s.status || {} : {};
  const pos = st.position_mm;
  $("tPosition").firstChild.textContent = typeof pos === "number" ? pos.toFixed(2) : "—";
  $("statusAge").textContent = c && s.status_age_s !== null ? `updated ${s.status_age_s.toFixed(1)} s ago` : "";
  const speed = st.speed_mm_s;
  $("tSpeed").textContent = c && typeof speed === "number" ? `${speed.toFixed(1)} ${U()}/s` : "";
  renderLamps(st, c);
  renderPush(c ? s.push : null);
  tile("tileLink", !c ? "—" : s.connection.simulate ? "Simulator" : `${s.connection.baud} baud`, c && s.comm_ok ? "on" : null);
  const code = st.alarm_code;
  let alarmText =
    code === undefined || code === null ? "—" : code === 0 ? "None" : typeof code === "number" ? "0x" + code.toString(16).toUpperCase().padStart(3, "0") : String(code);
  // The driver flags a code that only has the warning bit as alarm=false; show it, but not as an alarm.
  const alarmActive = typeof st.alarm === "boolean" ? st.alarm : alarmText !== "None" && alarmText !== "—";
  if (!alarmActive && alarmText !== "None" && alarmText !== "—") alarmText += " (warning)";
  tile("tileAlarm", alarmText, alarmActive ? "alert" : null);
  tile("tileModbus", yesNo(st.modbus_control, "Yes", "No (PIO)"), st.modbus_control ? "on" : null);
  renderStopWarning(s, st);
  renderControlMode(s, st);
  renderReset(s, st);

  if (c && !wasConnected) {
    loadIdentity();
    loadPositions();
  }
  if (!c && wasConnected) {
    renderIdentity(undefined);
    $("quickMoves").replaceChildren(el("span", "Connect to load positions 0-2.", "small"));
    $("storedMoves").replaceChildren(el("span", "Connect to load positions 0-2.", "small"));
  }
  wasConnected = c;
  renderEvents(s.events || []);
}

function renderEvents(events) {
  const key = events.length ? events[0].ts + ":" + events.length : "";
  if (key === lastEventKey) return;
  lastEventKey = key;
  const list = $("events");
  list.replaceChildren();
  for (const ev of events) {
    const li = el("li", undefined, ev.level);
    li.append(el("time", ev.t), document.createTextNode(ev.msg));
    list.append(li);
  }
}

async function poll() {
  let delay = POLL_MS;
  try {
    last = await api("/api/status");
    render(last);
  } catch (e) {
    const badge = $("connBadge");
    badge.className = "badge bad";
    badge.textContent = "GUI server unreachable";
    delay = 2000; // back off while the server is down
  } finally {
    setTimeout(poll, delay);
  }
}

poll();
