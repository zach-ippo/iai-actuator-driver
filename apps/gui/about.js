// About panel, demo-mode banner and Quit button, shared by the workbench ("/") and the single-axis page ("/classic").
// Reads /api/about (version, notice, evidence words, document links); the Quit button posts /api/quit.
"use strict";

(function () {
  const btn = document.getElementById("aboutBtn");
  if (!window.ECToken) return;
  const TOKEN = window.ECToken.get();
  let about = null;
  let dlg = null;

  function node(tag, attrs, ...kids) {
    const el = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs || {})) {
      if (v === undefined || v === null || v === false) continue;
      if (k === "class") el.className = v;
      else if (k === "text") el.textContent = v;
      else el.setAttribute(k, String(v));
    }
    for (const kid of kids.flat(Infinity)) {
      if (kid === null || kid === undefined || kid === false) continue;
      el.append(kid instanceof Node ? kid : document.createTextNode(String(kid)));
    }
    return el;
  }

  async function load() {
    if (about) return about;
    const r = await fetch("/api/about", { headers: { "X-EC-Token": TOKEN } });
    const v = await r.json();
    if (!r.ok || v.ok === false) throw Error(v.error || `Request failed (${r.status})`);
    about = v;
    return v;
  }

  function build(a) {
    const close = node("button", { type: "submit", value: "close", class: "primary", text: "Close" });
    const docs = a.docs.map((d) =>
      node("li", {}, d.available
        ? node("a", { href: d.href, target: "_blank", rel: "noopener", text: d.title })
        : node("span", { class: "muted", text: `${d.title} (not part of this install)` })));
    const dialog = node("dialog", { id: "aboutDlg", class: "about", "aria-labelledby": "aboutTitle" },
      node("form", { method: "dialog", class: "about-body" },
        node("h3", { id: "aboutTitle", text: `${a.product} ${a.version}` }),
        node("p", { class: "about-stage", text: `${a.stage} release - for ELECYLINDER and CON-family units over RS-485 Modbus RTU` }),
        a.notice.map((t, i) => node("p", { class: i === 0 ? "about-notice" : "about-note", text: t })),
        node("h4", { text: "Evidence words" }),
        node("dl", { class: "about-evidence" }, a.evidence.map(([w, t]) => [node("dt", { text: w }), node("dd", { text: t })])),
        node("h4", { text: "Documentation" }),
        node("ul", { class: "about-links" }, docs),
        node("p", { class: "small", id: "aboutMode",
          text: `${a.sim_only ? "Demo mode: simulators only, no serial port is listed or opened. " : ""}Python ${a.python}` +
            `${a.run_as ? ` (${a.run_as})` : ""}. The server listens on 127.0.0.1 only.` }),
        node("menu", {}, close)));
    document.body.append(dialog);
    return dialog;
  }

  function banner(a) {
    if (!a.sim_only || document.getElementById("demoBanner")) return;
    const bar = node("div", { id: "demoBanner", class: "demo-banner", role: "note",
      text: "DEMO MODE - simulators only. No serial port is listed or opened." });
    const head = document.querySelector("header.appbar");
    if (head) head.after(bar);
    else document.body.prepend(bar);
    const sim = document.getElementById("simulate"); // single-axis page: keep "Simulate" on
    if (sim) {
      sim.checked = true;
      sim.disabled = true;
      sim.title = "Demo mode (--sim-only): only the simulator can be used";
    }
  }

  if (btn) {
    btn.addEventListener("click", async () => {
      try {
        const a = await load();
        if (!dlg) dlg = build(a);
        dlg.showModal();
      } catch (e) {
        alert(`About: ${e.message}`);
      }
    });
  }
  load().then(banner).catch(() => {}); // the pages work without it

  // Quit: the server stops every axis, leaves each unit as found and ends. (Closing the console window does the same on
  // Windows, and so does Ctrl+C; the button is for a program that was started without a window in view.)
  const quit = document.getElementById("quitBtn");
  if (quit) {
    quit.addEventListener("click", async () => {
      if (!window.confirm("Quit the GUI program?\n\nEvery axis is stopped and each unit is left as found (control handed "
        + "back to PIO where it had it). This page stops working; start iai-gui again to continue.")) return;
      quit.disabled = true;
      quit.textContent = "Quitting...";
      try {
        const r = await fetch("/api/quit", { method: "POST", headers: { "X-EC-Token": TOKEN, "Content-Type": "application/json" }, body: "{}" });
        const v = await r.json();
        if (!r.ok || v.ok === false) throw Error(v.error || `Request failed (${r.status})`);
      } catch (e) {
        quit.disabled = false;
        quit.textContent = "Quit";
        alert(`Quit: ${e.message}`);
        return;
      }
      document.body.replaceChildren(node("main", { class: "stopped-note" },
        node("h2", { text: "GUI stopped" }),
        node("p", { text: "Every axis was stopped and each unit was left as found. You can close this tab." })));
    });
  }
})();
