"use strict";
// User-parameter Edit buttons for the Unit info "User parameters" panel. THIS FILE WRITES TO THE EC:
// after a confirmation it sends POST /api/param, which writes the parameter, verifies it by read-back
// and commits it to non-volatile memory with PARW (coil 0x0428), the commit frame seen on the wire.
// unitinfo.js stays read-only: it only asks paramEditButton() for the button of a row.
// Uses app.js globals: $, el, api, toast; unitinfo.js: refreshInfo.

const PARAM_PERMANENT_NOTE =
  "This is a PERMANENT write: the value is committed to the EC's non-volatile memory (PARW coil 0x0428). " +
  "The EC applies it only after a SOFTWARE RESET or a power cycle (Operate tab > Software reset).";
const PARAM_HOME_NOTE =
  "WARNING - home parameter (P3 / P4): after the reset an absolute (WA) unit comes up with alarm 0x0EE " +
  "'ABS error (2)' (home lost) and an incremental unit has lost its home anyway. Either way: Alarm reset, then Home, " +
  "before the next move.";

// The Edit control of one decoded parameter row (q: unit_info parameters.named[i]).
function paramEditButton(q) {
  if (!q.writable) {
    const why = !q.found ? "" : q.model_note || "not writable";
    return el("span", why, "small");
  }
  const b = el("button", "Edit", "needs-conn");
  b.type = "button";
  b.title = `Write P${q.no} (committed to non-volatile memory; applies after a software reset)` +
    (q.home_parameter ? ". HOME parameter: needs a home return afterwards." : "") +
    (q.listed === false && q.model_note ? `. Note: ${q.model_note}.` : "");
  b.disabled = !(typeof last !== "undefined" && last && last.connected);
  b.addEventListener("click", () => editParam(q));
  return b;
}

function choiceHint(q) {
  const labels = Object.values(q.choices || {});
  return labels.length ? `one of: ${labels.join(", ")} (the number or the label)` : `a value in ${q.unit || "mm"}`;
}

async function editParam(q) {
  const current = q.unit === "mm" && typeof q.value === "number" ? q.value.toFixed(2) : q.text;
  const text = prompt(`P${q.no} ${q.name}\nCurrent value: ${q.text}\n\nNew value - ${choiceHint(q)}:`, current);
  if (text === null) return;
  const value = text.trim();
  if (value === "") return toast("No value entered: nothing written.");
  let message = `Write P${q.no} ${q.name} = ${value} (now ${q.text})?\n\n${PARAM_PERMANENT_NOTE}`;
  if (q.home_parameter) message += `\n\n${PARAM_HOME_NOTE}`;
  // The unit's list does not show it: no gate, the warning goes in the same dialog.
  if (q.listed === false && q.model_note) message += `\n\nNote: ${q.model_note}; it is written anyway.`;
  if (!confirm(message)) return; // one plain OK / Cancel; a home parameter carries its warning in the same dialog
  try {
    const body = { no: q.no, value };
    const r = (await api("/api/param", body)).result;
    toast(
      r.skipped
        ? `P${r.no} = ${r.after}: ${r.note}.`
        : r.commit_confirmed === false
        ? `P${r.no}: ${r.before} -> ${r.after}, but the commit was NOT CONFIRMED (the EC never showed it busy): it may be RAM only.`
        : `P${r.no}: ${r.before} -> ${r.after}, committed. It takes effect after a software reset or a power cycle.`,
    );
  } catch (e) {
    toast(`P${q.no}: ${e.message}`);
    return;
  }
  if (typeof refreshInfo === "function") refreshInfo();
}
