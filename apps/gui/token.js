// The access key of this run, for every page script (window.ECToken.get()).
// The launch address carries it after the "#": http://127.0.0.1:8780/#k=<key>. A fragment is never sent to a
// server, written to a log or put in a Referer, and the served pages do not contain the key, so another account on
// this PC that fetches the page learns nothing. It is read once, kept in this tab's sessionStorage (a reload keeps
// working; another tab needs the address printed in the console) and removed from the address bar.
"use strict";

(function () {
  const STORE = "iaigui.key";
  let key = "";
  try {
    const found = /(?:^|[#&])k=([A-Za-z0-9_-]{8,128})(?:&|$)/.exec(window.location.hash);
    if (found) {
      key = found[1];
      try {
        window.sessionStorage.setItem(STORE, key);
      } catch {
        // no storage (a private window): the key stays in this page only
      }
      window.history.replaceState(null, "", window.location.pathname + window.location.search);
    }
  } catch {
    // no readable address: handled below as a missing key
  }
  if (!key) {
    try {
      key = window.sessionStorage.getItem(STORE) || "";
    } catch {
      key = "";
    }
  }
  window.ECToken = { get: () => key, present: () => key !== "" };

  if (!key) {
    document.addEventListener("DOMContentLoaded", () => {
      const note = document.createElement("div");
      note.className = "token-missing";
      note.setAttribute("role", "alert");
      note.textContent = "This page has no access key, so it cannot talk to the GUI program. Open the address printed in "
        + "the console window where iai-gui runs (it ends with #k=...), or start iai-gui again.";
      document.body.prepend(note);
    });
  }
})();
