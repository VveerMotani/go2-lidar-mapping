// Go2 dashboard v2 -- panel wiring.
//
// Polls /data and paints it. Every number on the page comes from that one JSON
// document; docs/dashboard.md lists which DDS topic and field each one is.
//
// The 3D view lives in robot3d.js and redraws every animation frame; this file
// only hands it fresh joint angles whenever /data returns.

import { Robot3D } from "/static/robot3d.js";
import { Charts, CHARTS } from "/static/charts.js";

const $ = (id) => document.getElementById(id);
const DEG = 180 / Math.PI;

// ---------------------------------------------------------------------------
// Settings. Remembered in localStorage so a browser reload keeps your layout.
// ---------------------------------------------------------------------------
const PANELS = [
  ["panel-model", "3D robot"],
  ["panel-cameras", "Cameras"],
  ["panel-power", "Power"],
  ["panel-attitude", "Orientation"],
  ["panel-motion", "Motion"],
  ["panel-feet", "Foot load"],
  ["panel-rates", "Streams"],
  ["panel-sensors", "Sensors"],
  ["panel-record", "Recording"],
  ["panel-walk", "Record a walk"],
  ["panel-library", "Walk library"],
  ["panel-joints", "Joints"],
];
const DEFAULTS = {
  camhost: "", hz: 5, units: "deg", theme: "dark",
  colorMode: "temperature",
  panels: Object.fromEntries(PANELS.map(([id]) => [id, true])),
  // Each graph inside the History panel can be hidden on its own, separately
  // from hiding the whole panel.
  charts: Object.fromEntries(CHARTS.map((c) => [c.id, true])),
};

let cfg = load();
// True when this browser has never saved settings of its own -- the only case
// in which the server's shared defaults are allowed to apply. After that, what
// you chose here wins, and the server copy is only a starting point for the
// next machine.
const cfgIsFresh = !localStorage.getItem("go2.dashboard");

function load() {
  try {
    const raw = JSON.parse(localStorage.getItem("go2.dashboard") || "{}");
    return {
      ...DEFAULTS, ...raw,
      panels: { ...DEFAULTS.panels, ...(raw.panels || {}) },
      charts: { ...DEFAULTS.charts, ...(raw.charts || {}) },
    };
  } catch (e) {
    return { ...DEFAULTS };
  }
}
function save() {
  try { localStorage.setItem("go2.dashboard", JSON.stringify(cfg)); } catch (e) {}
}

// ---------------------------------------------------------------------------
// Formatting helpers
// ---------------------------------------------------------------------------
const ang = (rad) =>
  cfg.units === "deg" ? (rad * DEG).toFixed(1) + "°" : rad.toFixed(3);
const vec = (a, unit, n = 2) =>
  a.map((v) => v.toFixed(n)).join("  ") + (unit ? " " + unit : "");

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------
const robot = new Robot3D($("canvas"), $("tooltip"));
robot.load()
  .then(() => robot.setColorMode(cfg.colorMode))
  .catch((e) => {
    // A failure here leaves a blank canvas and no other clue, so say so.
    console.error("3D model failed to load", e);
    $("status").textContent = "3D model failed to load: " + e;
  });
window.robot = robot;                 // handy from the browser console

const charts = new Charts();
applyTheme();
applyPanels();
applyCharts();
applyCameras();

if ($("btn-clear-history")) $("btn-clear-history").onclick = () => charts.clear();

$("set-charts").innerHTML = CHARTS.map((c) =>
  `<label><input type="checkbox" data-chart="${c.id}"
     ${cfg.charts[c.id] ? "checked" : ""}> ${c.label}</label>`).join("");
for (const cb of $("set-charts").querySelectorAll("input")) {
  cb.onchange = () => {
    cfg.charts[cb.dataset.chart] = cb.checked; save(); applyCharts();
  };
}

function applyCharts() {
  for (const c of CHARTS) charts.setVisible(c.id, cfg.charts[c.id]);
}
$("color-mode").value = cfg.colorMode;
updateLegend();

$("btn-reset").onclick = () => robot.resetView();
$("btn-top").onclick = () => robot.topView();
$("btn-side").onclick = () => robot.sideView();
$("color-mode").onchange = (e) => {
  cfg.colorMode = e.target.value;
  robot.setColorMode(cfg.colorMode);
  updateLegend();
  save();
};

function updateLegend() {
  const m = cfg.colorMode;
  $("legend-lo").textContent = m === "torque" ? "0 N·m" : m === "off" ? "" : "20°C";
  $("legend-hi").textContent = m === "torque" ? "25 N·m" : m === "off" ? "" : "80°C";
}

// ---------------------------------------------------------------------------
// Where other services live
// ---------------------------------------------------------------------------
// A host setting may be the literal "auto" (or "auto:PORT"), meaning "whatever
// address you reached this page on". That has to be worked out here rather than
// on the server, because the server has several addresses and only the browser
// knows which one you actually typed.
//
// This is what makes the dashboard work over Wi-Fi. The page is served by the
// Jetson, and so are the cameras -- but the camera host used to be the literal
// wired address, so over Wi-Fi the page loaded and the video panels were dead.
function resolveHost(value, fallbackPort) {
  const v = (value || "").trim();
  if (!v) return "";
  if (v === "auto" || v.startsWith("auto:")) {
    const port = v.includes(":") ? v.split(":")[1] : (fallbackPort || "8080");
    return `${location.hostname}:${port}`;
  }
  return v;
}

// ---------------------------------------------------------------------------
// Settings drawer
// ---------------------------------------------------------------------------
$("btn-settings").onclick = () => $("drawer").classList.add("open");
$("btn-close").onclick = () => $("drawer").classList.remove("open");
{
  const d = window.GO2_CAMERA_HOST;
  const r = resolveHost(d);
  $("camhost-default").textContent = d === r ? d : `${d} → ${r}`;
}

$("set-camhost").value = cfg.camhost;
$("set-hz").value = cfg.hz;
$("set-units").value = cfg.units;
$("set-theme").value = cfg.theme;

$("set-camhost").onchange = (e) => {
  cfg.camhost = e.target.value.trim(); save(); applyCameras();
};
$("set-hz").onchange = (e) => {
  cfg.hz = Math.max(1, Math.min(30, +e.target.value || 5)); save(); restartPolling();
};
$("set-units").onchange = (e) => {
  // No need to touch the joints table header here: paint() rebuilds that
  // table on every frame and reads cfg.units when it does. (It used to be set
  // here, which threw once the first frame had replaced the element.)
  cfg.units = e.target.value; save();
};
$("set-theme").onchange = (e) => { cfg.theme = e.target.value; save(); applyTheme(); };

$("set-panels").innerHTML = PANELS.map(([id, label]) =>
  `<label><input type="checkbox" data-panel="${id}"
     ${cfg.panels[id] ? "checked" : ""}> ${label}</label>`).join("");
for (const cb of $("set-panels").querySelectorAll("input")) {
  cb.onchange = () => {
    cfg.panels[cb.dataset.panel] = cb.checked; save(); applyPanels();
  };
}

function applyTheme() {
  document.documentElement.dataset.theme = cfg.theme;
  // The charts read their grid and label colours from the CSS variables, so
  // they have to be repainted when those change.
  if (typeof charts !== "undefined") charts.draw();
}

function applyPanels() {
  for (const [id] of PANELS) {
    const el = $(id);
    if (el) el.classList.toggle("hidden", !cfg.panels[id]);
  }
  // If both top-row panels are hidden the row should not hold empty space.
  const top = document.querySelector(".top");
  const shown = cfg.panels["panel-model"] || cfg.panels["panel-cameras"];
  top.style.display = shown ? "" : "none";
  top.style.gridTemplateColumns =
    cfg.panels["panel-model"] && cfg.panels["panel-cameras"] ? "" : "1fr";
}

function applyCameras() {
  // One setting drives both feeds. The server injected its own default into
  // the page; the drawer can override it without restarting anything.
  const host = resolveHost(cfg.camhost || window.GO2_CAMERA_HOST);
  for (const [id, cam] of [["cam2", 2], ["cam4", 4]]) {
    const img = $(id);
    img.onerror = () => {
      // A dead feed otherwise shows a broken-image icon, which reads like a
      // bug in this page rather than an unreachable camera server.
      const fig = img.parentElement;
      if (!fig.querySelector(".camfail")) {
        const p = document.createElement("div");
        p.className = "camfail";
        p.innerHTML = `no feed from <b>${host}</b><br>` +
          `<span>is camera_server.py running on the Jetson?</span>`;
        fig.appendChild(p);
      }
    };
    img.onload = () => {
      const p = img.parentElement.querySelector(".camfail");
      if (p) p.remove();
    };
    img.src = `http://${host}/stream?cam=${cam}`;
  }
}


// ---------------------------------------------------------------------------
// Recording controls
//
// The server does the recording; this only drives it. Every action refreshes
// the panel from what the server reports back, so the page never shows a state
// it merely assumes.
// ---------------------------------------------------------------------------
async function control(action, extra) {
  const body = Object.assign({ action }, extra || {});
  const res = await fetch("/api/control", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const data = await res.json().catch(() => ({}));
  recGeneration++;                  // any in-flight refresh is now out of date
  if (!res.ok) throw new Error(data.error || ("request failed: " + res.status));
  return data;
}

function fmtBytes(n) {
  return n > 1e6 ? (n / 1e6).toFixed(1) + " MB" : Math.round(n / 1e3) + " kB";
}
function fmtAgo(unixSeconds) {
  const mins = (Date.now() / 1000 - unixSeconds) / 60;
  if (mins < 1) return "just now";
  if (mins < 60) return Math.round(mins) + " min ago";
  const hrs = mins / 60;
  if (hrs < 24) return Math.round(hrs) + " h ago";
  return Math.round(hrs / 24) + " d ago";
}

let recState = null;

// The background refresh below runs every couple of seconds. A refresh started
// just before you press a button will answer just after it, describing the
// state as it was BEFORE the press -- which would repaint the panel with stale
// values and, worse, disable the button you were about to use next. Each
// control action bumps this counter, and a refresh whose fetch began before
// the latest action is discarded.
let recGeneration = 0;

function paintRecording(st) {
  recState = st;
  const rec = st.recording, rep = st.replaying;

  $("rec-mode").textContent = rec ? "RECORDING" : rep ? "PLAYING A RECORDING" : "LIVE";
  $("rec-mode").className = "rec-mode " + (rec ? "on" : rep ? "replay" : "live");

  $("btn-record").textContent = rec ? "Stop recording" : "Start recording";
  $("btn-record").classList.toggle("danger", !!rec);
  $("rec-name").disabled = !!rec;

  if (rec) {
    $("rec-status").textContent =
      `Saving to ${rec.file} — ${rec.frames} frames, ${rec.seconds.toFixed(0)} s.`;
  } else if (!st.can_go_live) {
    $("rec-status").textContent =
      "This server was started in playback mode, so it is not listening to a " +
      "robot and cannot record.";
  } else {
    $("rec-status").textContent =
      "Saves what the robot is doing to a file, so you can play it back later " +
      "without the robot.";
  }
  $("btn-record").disabled = !st.can_go_live || !!rep;

  // The list of past recordings. Keep the current choice selected across
  // refreshes so the dropdown does not jump under the pointer.
  const sel = $("rec-file");
  const keep = sel.value;
  if (!st.files.length) {
    sel.innerHTML = '<option value="">no recordings yet</option>';
  } else {
    sel.innerHTML = st.files.map((f) =>
      `<option value="${f.name}">${f.name} — ${f.duration.toFixed(0)}s</option>`).join("");
    if (keep && st.files.some((f) => f.name === keep)) sel.value = keep;
    else if (rep) sel.value = rep.file;
  }
  const chosen = st.files.find((f) => f.name === sel.value);
  $("rec-files").textContent = chosen
    ? `${chosen.frames} frames · ${fmtBytes(chosen.bytes)} · saved ${fmtAgo(chosen.modified)}`
    : `Saved in ${st.dir}`;

  $("btn-play").disabled = !st.files.length || !!rec;
  $("btn-live").disabled = !rep || !st.can_go_live;

  paintWalk(st);

  // Deleting is the one action here that cannot be undone, so it takes two
  // clicks: the first arms it and names the file, the second does it. The
  // armed state clears on its own after a few seconds, and whenever the
  // selection changes, so a stale "Confirm" can never delete the wrong file.
  const del = $("btn-delete");
  del.disabled = !st.files.length || !!rec ||
                 (!!rep && rep.file === sel.value);
  if (deleteArmed !== sel.value) {
    del.textContent = "Delete";
    del.classList.remove("armed");
  }
}

let deleteArmed = null;
let deleteTimer = null;

function disarmDelete() {
  deleteArmed = null;
  clearTimeout(deleteTimer);
  const del = $("btn-delete");
  del.textContent = "Delete";
  del.classList.remove("armed");
}

async function refreshRecording() {
  const gen = recGeneration;
  try {
    const st = await (await fetch("/api/recordings", { cache: "no-store" })).json();
    if (gen !== recGeneration) return;    // a button was pressed while we waited
    paintRecording(st);
  } catch (e) { /* the poll loop already reports an unreachable server */ }
}

// ---------------------------------------------------------------------------
// Walk recording -- the big one: raw LiDAR plus the IMU, for building a map.
// A different thing from the dashboard recording above, and about 160x larger,
// so it gets its own panel and its own warnings.
// ---------------------------------------------------------------------------
let walkDeleteArmed = null;
let walkDeleteTimer = null;

function disarmWalkDelete() {
  walkDeleteArmed = null;
  clearTimeout(walkDeleteTimer);
  const b = $("btn-walk-delete");
  b.textContent = "Delete";
  b.classList.remove("armed");
}

function paintWalk(st) {
  const w = st.walk;
  if (!w) return;
  const rec = w.recording && w.recording.running ? w.recording : null;

  paintWalkProgress(rec);
  paintVerdict(w.last_verdict);
  // The pre-flight strip is only meaningful where a walk can actually be
  // recorded. On the laptop the recorder is not installed and the whole panel
  // is inert, so a permanently red strip there would be noise.
  $("pf").classList.toggle("hidden", !w.available || !!rec);

  $("walk-mode").textContent = rec ? "RECORDING" : w.available ? "READY" : "UNAVAILABLE";
  $("walk-mode").className = "rec-mode " + (rec ? "on" : w.available ? "live" : "");

  $("btn-walk").textContent = rec ? "Stop walk" : "Start walk";
  $("btn-walk").classList.toggle("danger", !!rec);
  $("btn-walk").disabled = !w.available;
  $("walk-name").disabled = !!rec || !w.available;

  if (rec) {
    const mb = (rec.lidar_bytes || 0) / 1e6;
    $("walk-status").textContent =
      `${rec.name} — ${rec.seconds.toFixed(0)} s · ` +
      `${(rec.lidar_packets || 0).toLocaleString()} lidar packets, ${mb.toFixed(0)} MB · ` +
      `${(rec.imu_rows || 0).toLocaleString()} imu rows at ${rec.imu_hz || 0} Hz`;
  } else if (!w.available) {
    $("walk-status").textContent =
      "record_session.py is not on this machine. Copy it next to dashboard.py " +
      "on the robot to enable this.";
  } else {
    $("walk-status").textContent =
      "Saves the raw LiDAR and the IMU for building a map later. " +
      "About 160 MB a minute.";
  }

  const sel = $("walk-file");
  const keep = sel.value;
  if (!w.walks.length) {
    sel.innerHTML = '<option value="">no walks yet</option>';
  } else {
    sel.innerHTML = w.walks.map((k) => {
      const mb = k.bytes / 1e6;
      const size = mb > 1000 ? (mb / 1000).toFixed(1) + " GB" : mb.toFixed(0) + " MB";
      // A walk that is recording right now has no session.json yet. Saying
      // "incomplete" about it would read as a fault rather than as progress.
      const live = rec && rec.name === k.name;
      const note = live ? " (recording now)" : k.complete ? "" : " (incomplete)";
      const secs = live ? rec.seconds : k.seconds;
      return `<option value="${k.name}">${k.name} — ${secs.toFixed(0)}s, ${size}` +
             `${note}</option>`;
    }).join("");
    if (keep && w.walks.some((k) => k.name === keep)) sel.value = keep;
  }

  const chosen = w.walks.find((k) => k.name === sel.value);
  if (chosen && rec && rec.name === chosen.name) {
    $("walk-files").textContent = "Recording now — its totals appear when it stops.";
  } else if (chosen) {
    const bits = [`${(chosen.lidar_packets || 0).toLocaleString()} lidar packets`,
                  `${(chosen.imu_rows || 0).toLocaleString()} imu rows`];
    // A walk recorded before the robot's clock was set cannot be lined up with
    // anything, so say so here rather than letting it be discovered later.
    if (chosen.complete && chosen.clock_ok === false) {
      bits.push("CLOCK WAS NOT SYNCED — not usable for mapping");
    }
    if (!chosen.complete) bits.push("no session.json — it did not finish cleanly");
    $("walk-files").textContent = bits.join(" · ");
  } else {
    $("walk-files").textContent = `Saved in ${w.dir}`;
  }

  $("btn-walk-delete").disabled = !w.walks.length ||
    (rec && rec.name === sel.value);

  // Download: the walk folder as a zip, straight off the robot's disk. This is
  // what fetch_walk.sh does over scp, for a laptop that has only a browser.
  // Not while it is recording -- the files are still growing.
  const dl = $("btn-walk-download");
  const canDl = !!chosen && !(rec && rec.name === chosen.name);
  dl.classList.toggle("disabled", !canDl);
  if (canDl) {
    dl.href = "/api/walk-download?name=" + encodeURIComponent(chosen.name);
    dl.setAttribute("download", chosen.name + ".zip");
  } else {
    dl.removeAttribute("href");
  }
  if (walkDeleteArmed !== sel.value) {
    $("btn-walk-delete").textContent = "Delete";
    $("btn-walk-delete").classList.remove("armed");
  }
}

$("btn-walk").onclick = async () => {
  const rec = recState && recState.walk && recState.walk.recording;
  try {
    if (rec && rec.running) {
      $("walk-status").textContent = "stopping, finishing the file…";
      paintRecording(await control("walk-stop"));
    } else {
      $("walk-status").textContent = "starting…";
      const name = $("walk-name").value.trim();
      try {
        paintRecording(await control("walk-start", { name }));
      } catch (e) {
        // The server refuses when a blocking check fails. Say which, and
        // offer to go anyway -- it is occasionally right to, and a check that
        // cannot be overridden gets worked around in worse ways.
        if (!/^not ready: /.test(e.message)) throw e;
        await refreshPreflight();
        const go = confirm(
          `${e.message}\n\nRecord anyway?\n\n` +
          "These are the faults that spoiled both previous walks, and most of " +
          "them cannot be fixed after the fact.");
        if (!go) { $("walk-status").textContent = e.message; return; }
        paintRecording(await control("walk-start", { name, force: true }));
      }
      $("walk-name").value = "";
    }
  } catch (e) { $("walk-status").textContent = "Could not do that: " + e.message; }
};

$("btn-walk-delete").onclick = async () => {
  const name = $("walk-file").value;
  if (!name) return;
  if (walkDeleteArmed !== name) {
    walkDeleteArmed = name;
    $("btn-walk-delete").textContent = "Confirm delete";
    $("btn-walk-delete").classList.add("armed");
    $("walk-files").textContent =
      `Delete the whole ${name} folder? This cannot be undone. Click again.`;
    clearTimeout(walkDeleteTimer);
    walkDeleteTimer = setTimeout(disarmWalkDelete, 6000);
    return;
  }
  disarmWalkDelete();
  try {
    paintRecording(await control("walk-delete", { name }));
  } catch (e) { $("walk-files").textContent = "Could not delete: " + e.message; }
};

$("btn-walk-download").onclick = () => {
  const name = $("walk-file").value;
  // No size is known in advance -- the zip is made as it is sent -- so the
  // browser's download list shows bytes received rather than a percentage.
  if (name) $("walk-files").textContent =
    `Downloading ${name}.zip — see the browser's downloads. ` +
    "The page keeps working meanwhile; keep the cable in until it finishes.";
};

$("walk-file").onchange = () => {
  disarmWalkDelete();
  if (recState) paintWalk(recState);
};

function recError(e) {
  $("rec-status").textContent = "Could not do that: " + e.message;
}

$("btn-record").onclick = async () => {
  try {
    if (recState && recState.recording) {
      paintRecording(await control("record-stop"));
    } else {
      paintRecording(await control("record-start", { name: $("rec-name").value.trim() }));
      $("rec-name").value = "";
    }
  } catch (e) { recError(e); }
};

$("btn-play").onclick = async () => {
  const file = $("rec-file").value;
  if (!file) return;
  try {
    charts.clear();          // the graphs were tracking a different source
    paintRecording(await control("replay", { file }));
  } catch (e) { recError(e); }
};

$("btn-live").onclick = async () => {
  try {
    charts.clear();
    paintRecording(await control("live"));
  } catch (e) { recError(e); }
};

$("btn-delete").onclick = async () => {
  const file = $("rec-file").value;
  if (!file) return;

  if (deleteArmed !== file) {
    deleteArmed = file;
    $("btn-delete").textContent = "Confirm delete";
    $("btn-delete").classList.add("armed");
    $("rec-status").textContent =
      `Delete ${file}? This cannot be undone. Click again to confirm.`;
    clearTimeout(deleteTimer);
    deleteTimer = setTimeout(disarmDelete, 6000);
    return;
  }

  disarmDelete();
  try {
    const st = await control("delete", { file });
    paintRecording(st);
    $("rec-status").textContent = `Deleted ${file}.`;
  } catch (e) { recError(e); }
};

$("rec-file").onchange = () => {
  disarmDelete();
  if (recState) paintRecording(recState);
};

refreshRecording();
setInterval(refreshRecording, 2000);

// ---------------------------------------------------------------------------
// Polling
// ---------------------------------------------------------------------------
let timer = null;
function restartPolling() {
  if (timer) clearInterval(timer);
  timer = setInterval(tick, Math.round(1000 / cfg.hz));
}

async function tick() {
  let d;
  try {
    d = await (await fetch("/data", { cache: "no-store" })).json();
  } catch (e) {
    $("dot").className = "dot";
    $("status").textContent = "dashboard server unreachable";
    $("banner").classList.add("show");
    $("banner").textContent = "NO DATA — cannot reach the dashboard server";
    return;
  }
  paint(d);
}

function paint(d) {
  // --- connection state ------------------------------------------------
  $("dot").className = "dot" + (d.connected ? " live" : "");
  const src = d.source && d.source.lowstate;
  $("status").textContent = d.connected
    ? `receiving ${src} at ${d.rates.lowstate.toFixed(0)} Hz`
    : "waiting for the first message…";

  const banner = $("banner");
  banner.classList.toggle("show", !!d.stale);
  if (d.stale) {
    banner.textContent = d.age === null
      ? "NO DATA — nothing has arrived yet. Is the robot on and the cable in?"
      : `NO DATA — nothing for ${d.age.toFixed(1)} s`;
  }

  // --- replay ----------------------------------------------------------
  // A recording must never be mistaken for a live robot, so say so loudly.
  const badge = $("replay-badge");
  if (d.replay) {
    badge.classList.remove("hidden");
    badge.textContent =
      `REPLAY · ${d.replay.file} · ${d.replay.position.toFixed(0)}/${d.replay.duration.toFixed(0)}s`;
  } else {
    badge.classList.add("hidden");
  }

  // --- 3D --------------------------------------------------------------
  robot.update(d);

  // --- graphs ----------------------------------------------------------
  charts.push(d);

  // --- power -----------------------------------------------------------
  if (d.power) {
    $("soc").textContent = d.power.soc + "%";
    $("socbar").style.width = d.power.soc + "%";
    $("socbar").style.background =
      d.power.soc < 20 ? "var(--bad)" : d.power.soc < 40 ? "var(--warn)" : "var(--accent)";
    $("volts").textContent = d.power.volts.toFixed(2) + " V";
    $("amps").textContent = d.power.amps.toFixed(2) + " A";
    $("ptemp").textContent = d.power.temp + " °C";
  }

  // --- orientation -----------------------------------------------------
  if (d.imu) {
    $("roll").textContent = ang(d.imu.roll);
    $("pitch").textContent = ang(d.imu.pitch);
    $("yaw").textContent = ang(d.imu.yaw);
    $("gyro").textContent = vec(d.imu.gyro, "rad/s");
    $("accel").textContent = vec(d.imu.accel, "m/s²");

    // Artificial horizon: roll turns the disc, pitch slides it. 2.2 px per
    // degree matches the 15 px pitch ladder drawn in the SVG.
    const rollDeg = d.imu.roll * DEG, pitchDeg = d.imu.pitch * DEG;
    $("horizon").setAttribute(
      "transform", `rotate(${-rollDeg}) translate(0 ${pitchDeg * 2.2})`);
    $("rollmark").setAttribute("transform", `rotate(${-rollDeg})`);
  }

  // --- motion ----------------------------------------------------------
  if (d.motion) {
    $("gait").textContent = d.motion.gait;
    $("mode").textContent = d.motion.mode_name;
    $("bh").textContent = d.motion.body_height.toFixed(3) + " m";
    $("vel").textContent = vec(d.motion.velocity, "m/s");
    $("yrate").textContent = d.motion.yaw_speed.toFixed(3) + " rad/s";
    $("pos").textContent = vec(d.motion.position, "m");
  }

  // --- foot load -------------------------------------------------------
  if (d.feet) {
    // Laid out as you look down on the robot from above: front row first,
    // left column on the left. The SDK order is FR, FL, RR, RL.
    const order = [1, 0, 3, 2];              // FL FR RL RR
    let total = 0;
    $("feet").innerHTML = order.map((i) => {
      const f = d.feet[i];
      total += f.force;
      const pct = Math.min(100, (f.force / 120) * 100);
      const contact = f.force > 15;
      return `<div class="foot ${contact ? "contact" : ""}">
        <div class="name">${f.name}${contact ? " — contact" : ""}</div>
        <div class="val">${f.force}</div>
        <div class="bar"><i style="width:${pct}%;background:${
          pct > 75 ? "var(--bad)" : pct > 40 ? "var(--warn)" : "var(--ok)"}"></i></div>
      </div>`;
    }).join("");

    // Sum of the four foot forces as a rough body-weight estimate. Units are
    // newtons against the simulator; on the robot foot_force is a raw sensor
    // count, so this reads as a relative load, not kilograms. See
    // docs/dashboard.md.
    const contacts = d.feet.filter((f) => f.force > 15).map((f) => f.name);
    $("weight").textContent =
      `total ${total} · ${contacts.length ? contacts.join(" ") : "airborne"}`;
  }

  // --- streams ---------------------------------------------------------
  if (d.rates) {
    // The server judges each stream against what it should be doing and says
    // so in words; this only lays it out.
    const streams = d.streams || [];
    $("stream-table").innerHTML =
      "<tr><th>stream</th><th>rate</th><th>what that means</th></tr>" +
      streams.map((st) =>
        `<tr><td>${st.label}</td><td>${st.hz.toFixed(0)} Hz</td>` +
        `<td class="note ${st.ok ? "" : "bad"}">${st.note}</td></tr>`).join("");
    $("age").textContent = d.age === null ? "never" : d.age.toFixed(2) + " s ago";
    $("uptime").textContent = d.uptime.toFixed(0) + " s";
  }

  // --- joints ----------------------------------------------------------
  if (d.joints) {
    const rows = d.joints.map((j) =>
      `<tr class="${j.temp > 70 ? "hot" : ""}"><td>${j.name}</td>
       <td>${ang(j.q)}</td><td>${j.dq.toFixed(2)}</td>
       <td>${j.tau.toFixed(2)}</td><td>${j.temp}°</td></tr>`).join("");
    $("joints").innerHTML =
      `<tr><th>joint</th><th>${cfg.units === "deg" ? "angle" : "angle (rad)"}</th>
       <th>velocity</th><th>torque</th><th>temp</th></tr>` + rows;
  }
}

// ---------------------------------------------------------------------------
// Demo mode
// ---------------------------------------------------------------------------
// A read-only view for when someone else is looking: the things worth watching,
// larger, and nothing clickable that could move the robot or delete a
// recording. Reached by adding ?demo to the address, so it cannot be entered by
// a stray click, and left with Escape.
//
// It is a presentation of the same page rather than a second page: the same
// polling, the same 3D view, the same numbers. A separate demo page would be
// one more thing to keep in step, and would be out of date the first time a
// panel changed.
function applyDemo(on) {
  document.documentElement.toggleAttribute("data-demo", !!on);
  if (on) $("drawer").classList.remove("open");
}

const demoWanted = new URLSearchParams(location.search).has("demo");
applyDemo(demoWanted);

document.addEventListener("keydown", (e) => {
  if (e.key !== "Escape") return;
  if (document.documentElement.hasAttribute("data-demo")) {
    // Leave by rewriting the address, so a reload does not put it back.
    const url = new URL(location.href);
    url.searchParams.delete("demo");
    history.replaceState({}, "", url);
    applyDemo(false);
  }
});

$("btn-demo").onclick = () => {
  const url = new URL(location.href);
  url.searchParams.set("demo", "");
  history.replaceState({}, "", url);
  applyDemo(true);
};

// ---------------------------------------------------------------------------
// Pre-flight
// ---------------------------------------------------------------------------
// Asked of the server every few seconds while the page is open, and again the
// moment before a walk starts. Everything it reports was knowable before the
// button was pressed; the two walks recorded so far were both spoiled by
// something in this list, and neither was noticed until the file was on
// another machine.
let pfState = null;
let pfSelected = null;

function chipRow(checks, cls, onPick) {
  return checks.map((c, i) =>
    `<span class="chip ${cls} ${c.ok ? "ok" : "bad"}" data-i="${i}"
       title="${(c.detail || "").replace(/"/g, "&quot;")}">${c.headline}</span>`
  ).join("");
}

function paintPreflight(pf) {
  pfState = pf;
  const box = $("pf");
  const title = $("pf-title");

  if (pf.ready) {
    title.textContent = `ready — ${pf.passed} of ${pf.total} checks`;
    title.className = "ready";
  } else {
    title.textContent = `not ready — ${pf.blockers.join(", ")}`;
    title.className = "blocked";
  }
  // A bar rather than a bare green light: several of these come good on their
  // own as the robot boots -- the motion service takes a few seconds to start
  // publishing -- and watching a number climb is easier than staring at a red
  // dot wondering whether anything is happening.
  const bar = $("pf-bar");
  bar.style.width = `${(pf.passed / pf.total * 100).toFixed(0)}%`;
  bar.classList.toggle("done", pf.ready);
  bar.classList.toggle("failed", !pf.ready);

  $("pf-chips").innerHTML = chipRow(pf.checks, "pf");
  addClockFix(pf);

  // Clicking a chip shows its detail below, because a tooltip is useless on a
  // phone and the phone is where you are standing when this matters.
  for (const el of $("pf-chips").querySelectorAll(".chip")) {
    el.onclick = () => {
      const c = pf.checks[+el.dataset.i];
      pfSelected = c.headline;
      $("pf-detail").textContent = `${c.headline}: ${c.detail}`;
      for (const o of $("pf-chips").querySelectorAll(".chip"))
        o.classList.toggle("sel", o === el);
    };
    if (pf.checks[+el.dataset.i].headline === pfSelected) el.click();
  }

  // With nothing chosen, show the first thing that is wrong -- that is what
  // you want to read, and it saves a click.
  if (!pfSelected) {
    const first = pf.checks.find((c) => !c.ok);
    $("pf-detail").textContent = first
      ? `${first.headline}: ${first.detail}`
      : "Everything checked out. Stand the robot where it can see structure, " +
        "start, wait ten seconds and confirm the counters are climbing.";
  }
}

// The clock chip is the one red chip a browser can fix by itself: the robot has
// no battery-backed clock and boots at 1970, and the laptop in front of you
// knows the time. This replaces go2-start.sh's `sudo date -s` over ssh, so a
// laptop with nothing but a browser can record a usable walk.
let clockSetting = false;

function addClockFix(pf) {
  const i = pf.checks.findIndex((c) => c.headline === "clock is set");
  if (i < 0 || pf.checks[i].ok) return;
  const chip = $("pf-chips").querySelector(`.chip[data-i="${i}"]`);
  if (!chip) return;
  const b = document.createElement("button");
  b.className = "chip-fix primary";
  b.textContent = clockSetting ? "setting…" : "Set from this computer";
  b.disabled = clockSetting;
  b.onclick = setClock;
  chip.after(b);
}

async function setClock() {
  const now = new Date();
  // A browser whose own clock is wrong would make things worse, not better.
  // The server refuses anything before 2026 too; this just says so sooner.
  if (now.getFullYear() < 2026) {
    $("pf-detail").textContent =
      `This computer's clock reads ${now.toLocaleString()} — set it first.`;
    return;
  }
  clockSetting = true;
  if (pfState) paintPreflight(pfState);
  try {
    const r = await control("set-clock", { epoch_ms: Date.now() });
    clockSetting = false;
    pfSelected = "clock is set";      // show what the robot reads now
    paintPreflight(r.preflight);
  } catch (e) {
    clockSetting = false;
    if (pfState) paintPreflight(pfState);
    $("pf-detail").textContent = "Could not set the clock: " + e.message;
  }
}

async function refreshPreflight() {
  try {
    paintPreflight(await (await fetch("/api/preflight",
                                      { cache: "no-store" })).json());
  } catch (e) { /* tick() already says the server is unreachable */ }
}

$("btn-pf-recheck").onclick = () => { pfSelected = null; refreshPreflight(); };

// ---------------------------------------------------------------------------
// While a walk is recording
// ---------------------------------------------------------------------------
function paintWalkProgress(rec) {
  const box = $("walk-progress");
  if (!rec) { box.classList.add("hidden"); return; }
  box.classList.remove("hidden");

  $("wp-name").textContent = rec.name;
  $("wp-elapsed").textContent =
    `${Math.floor(rec.seconds / 60)}m ${Math.round(rec.seconds % 60)}s · ` +
    `${((rec.bytes_on_disk || 0) / 1e6).toFixed(0)} MB`;

  const left = rec.minutes_left;
  $("wp-disk").textContent = left == null ? "" :
    `${(rec.disk_free / 1024 ** 3).toFixed(1)} GB free · about ${
      left.toFixed(0)} min of room`;

  // There is no total to count towards -- a walk ends when you stop it -- so
  // the bar shows how much of the ROOM has been used, which is the only
  // finite thing here and the only one that can end the recording for you.
  const used = rec.minutes_left != null && rec.seconds
    ? rec.seconds / 60 / (rec.seconds / 60 + rec.minutes_left) : 0;
  const bar = $("wp-bar");
  bar.style.width = `${Math.min(100, used * 100).toFixed(1)}%`;
  bar.classList.toggle("failed", (rec.health || []).some((h) => !h.ok));

  $("wp-health").className = "health";
  $("wp-health").innerHTML = (rec.health || []).map((h) =>
    `<div><b>${h.name}</b> <span class="${h.ok ? "ok" : "bad"}">${
      h.count.toLocaleString()} · ${h.note}</span></div>`).join("");
}

// ---------------------------------------------------------------------------
// What the walk turned out to be
// ---------------------------------------------------------------------------
function paintVerdict(v) {
  const box = $("walk-verdict");
  if (!v) { box.classList.add("hidden"); return; }
  box.classList.remove("hidden");

  const ok = v.verdict.ok;
  $("wv-title").textContent =
    `${v.name} — ${v.verdict.passed} of ${v.verdict.total} checks` +
    (ok ? ", good to convert" : "");
  $("wv-title").className = ok ? "ready" : "blocked";
  $("wv-chips").innerHTML = chipRow(v.verdict.checks, "pf");

  for (const el of $("wv-chips").querySelectorAll(".chip")) {
    el.onclick = () => {
      const c = v.verdict.checks[+el.dataset.i];
      $("wv-detail").textContent = `${c.headline}: ${c.detail}`;
      for (const o of $("wv-chips").querySelectorAll(".chip"))
        o.classList.toggle("sel", o === el);
    };
  }
  const first = v.verdict.checks.find((c) => !c.ok);
  $("wv-detail").textContent = first
    ? `${first.headline}: ${first.detail}`
    : "Nothing wrong with it. Copy it to the laptop and process it.";
}

// ---------------------------------------------------------------------------
// Settings kept on the server
// ---------------------------------------------------------------------------
// Two things come from /api/settings: values a fresh browser should start from,
// and facts about the robot that several programs must agree on -- the LiDAR's
// mounting offset in particular, which was a guess in three separate files
// until the vendor's number replaced it.
let serverSettings = null;

function fmtOffset(o) {
  if (!o) return "unknown";
  return `x ${o.x} m · y ${o.y} m · z ${o.z} m` +
         (o.yaw ? ` · yaw ${o.yaw} rad` : " · no rotation");
}

async function loadServerSettings() {
  try {
    const d = await (await fetch("/api/settings", { cache: "no-store" })).json();
    serverSettings = d.settings || {};
  } catch (e) {
    $("set-defaults-state").textContent = "the server did not answer";
    return;
  }

  $("set-offset").textContent = fmtOffset(serverSettings.lidar_offset);
  $("set-offset").title = (serverSettings.lidar_offset || {}).source || "";
  $("set-defaults-state").textContent = serverSettings.dashboard
    ? "this dashboard has saved defaults"
    : "no defaults saved yet — this browser is using the built-in ones";

  // Only on a first visit. A browser that has settings keeps them.
  if (cfgIsFresh && serverSettings.dashboard) {
    cfg = {
      ...cfg, ...serverSettings.dashboard,
      panels: { ...cfg.panels, ...(serverSettings.dashboard.panels || {}) },
      charts: { ...cfg.charts, ...(serverSettings.dashboard.charts || {}) },
    };
    save();
    applyTheme(); applyPanels(); applyCameras(); applyCharts();
    $("set-defaults-state").textContent =
      "started from this dashboard's saved defaults";
  }
}

$("btn-save-defaults").onclick = async () => {
  const btn = $("btn-save-defaults");
  btn.disabled = true;
  try {
    const r = await fetch("/api/control", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        action: "settings-save",
        // The mounting offset is carried through unchanged: it is the server's
        // fact, not a browser preference, and must not be lost by saving.
        settings: { ...(serverSettings || {}), dashboard: cfg },
      }),
    });
    const d = await r.json();
    if (!r.ok) throw new Error(d.error || "save failed");
    serverSettings = d.settings;
    $("set-defaults-state").textContent = `saved to ${d.path}`;
  } catch (e) {
    $("set-defaults-state").textContent = e.message;
  } finally {
    btn.disabled = false;
  }
};

// ---------------------------------------------------------------------------
// Which way in
// ---------------------------------------------------------------------------
// The same dashboard is reachable over the ethernet cable and, once the robot
// is on a wireless network, over Wi-Fi. Nothing on the page otherwise says
// which -- and the difference matters, because unplugging the cable is a normal
// part of recording a walk. So ask the server which addresses it answers to and
// match the one in the address bar against them.
//
// Re-asked every 15 s rather than once, because an interface can appear after
// the page is open: the robot's Wi-Fi may still be associating at boot.
async function showRoute() {
  const el = $("route");
  let h;
  try {
    h = await (await fetch("/health", { cache: "no-store" })).json();
  } catch (e) {
    return;                       // tick() already reports an unreachable server
  }
  const me = location.hostname;
  if (me === "localhost" || me === "127.0.0.1" || me === "::1") {
    el.textContent = "on this machine";
    el.title = `served by ${(h.host && h.host.hostname) || "this host"}`;
    return;
  }
  const list = (h.host && h.host.addresses) || [];
  const hit = list.find((a) => a.addr === me);
  const LABEL = { wireless: "over Wi-Fi", wired: "over the cable",
                  loopback: "on this machine" };
  el.textContent = hit ? `${me} ${LABEL[hit.kind] || "via " + hit.iface}` : me;
  // The tooltip lists every address the robot answers to, which is the fastest
  // way to find the other one when you want to switch.
  el.title = list.length
    ? "this server also answers to:" + list
        .filter((a) => a.kind !== "loopback")
        .map((a) => "\n  " + a.addr + "  (" + a.iface + ", " + a.kind + ")")
        .join("")
    : "";
}

// ---------------------------------------------------------------------------
// Walk library
// ---------------------------------------------------------------------------
// Lists the recordings on this machine, what is wrong with each, and turns one
// into a bag. Polled separately from /data and much more slowly: the list is
// read from disk and a walk is two hundred megabytes, so asking five times a
// second would be absurd. It speeds up while a conversion is running, because
// then there is a bar to move.
//
// The whole panel hides itself where it cannot work. On the robot the server is
// started without --walks, reports `available: false`, and says why -- turning
// a walk into a bag needs the Hesai driver, which is on the laptop.

let libTimer = null;
let libPeriodMs = 0;
let libBusy = false;

function libFmtSize(n) {
  if (!n) return "—";
  const u = ["B", "KB", "MB", "GB"];
  let i = 0;
  while (n >= 1024 && i < u.length - 1) { n /= 1024; i++; }
  return (i === 0 ? n.toFixed(0) : n.toFixed(1)) + " " + u[i];
}

function libFmtWhen(unix) {
  if (!unix) return "—";
  const d = new Date(unix * 1000);
  const pad = (v) => String(v).padStart(2, "0");
  return `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())} ` +
         `${pad(d.getHours())}:${pad(d.getMinutes())}`;
}

function libFmtLength(s) {
  if (!s) return "—";
  const m = Math.floor(s / 60);
  return m ? `${m}m ${Math.round(s % 60)}s` : `${Math.round(s)}s`;
}

// A /mnt/e/... path as Windows sees it. Foxglove Desktop is a Windows program
// and cannot open a WSL path, so the row shows the form you can actually paste.
function libWinPath(p) {
  const m = /^\/mnt\/([a-z])(\/.*)?$/.exec(p || "");
  if (!m) return p;
  return m[1].toUpperCase() + ":" + (m[2] || "/").replace(/\//g, "\\");
}

async function libPost(action, params) {
  const body = Object.assign({ action }, params || {});
  const r = await fetch("/api/control", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const d = await r.json();
  if (!r.ok) throw new Error(d.error || `request failed (${r.status})`);
  return d;
}

function libRenderJob(job) {
  const box = $("lib-job");
  if (!job) { box.classList.add("hidden"); return; }
  box.classList.remove("hidden");

  $("job-name").textContent = job.name;
  $("job-stage").textContent = job.stage_text || "";
  $("job-elapsed").textContent = libFmtLength(job.elapsed);

  const bar = $("job-bar");
  bar.style.width = `${job.percent}%`;
  bar.classList.toggle("failed", !job.running && job.returncode !== 0);
  bar.classList.toggle("done", !job.running && job.returncode === 0);

  $("btn-job-cancel").disabled = !job.running;
  $("btn-job-cancel").textContent = job.running ? "Cancel" : "Finished";
  $("job-log").textContent = (job.log || []).join("\n");
  $("job-log").scrollTop = $("job-log").scrollHeight;
}

function libRenderWalks(d) {
  const running = d.job && d.job.running ? d.job.name : null;
  const rows = (d.walks || []).map((w) => {
    const chips = w.verdict.checks.map((c) =>
      `<span class="chip ${c.ok ? "ok" : "bad"}" title="${
        (c.detail || "").replace(/"/g, "&quot;")}">${c.headline}</span>`).join("");

    // A missing bag is not a failure -- it just has not been converted yet --
    // so it reads as an invitation rather than a fault.
    const bag = w.bag.mcap
      ? `<span class="lib-path" title="paste this into Foxglove: File → Open local file">${
           libWinPath(w.bag.mcap)}</span>`
      : `<span class="rec-status">not converted yet</span>`;

    // The scan-matched map: a .ply for CloudCompare, and a Foxglove file in
    // which scrubbing shows everything mapped up to that moment.
    const map = w.map
      ? `<div class="lib-path" title="CloudCompare, MeshLab or Blender">${
           libWinPath(w.map.ply)} · ${libFmtSize(w.map.ply_bytes)}</div>` +
        (w.map.dense
          ? `<div class="lib-path" title="every scan, cleaned: CloudCompare">${
               libWinPath(w.map.dense)} · ${libFmtSize(w.map.dense_bytes)}</div>`
          : "") +
        (w.map.mcap
          ? `<div class="lib-path" title="Foxglove: display frame map, Follow mode Fixed">${
               libWinPath(w.map.mcap)}</div>`
          : "")
      : "";

    const busy = running === w.name;
    const label = w.bag.mcap ? "Re-process" : "Process";
    return `<tr>
      <td class="name">${w.name}</td>
      <td class="num">${libFmtWhen(w.started_unix)}</td>
      <td class="num">${libFmtLength(w.seconds)}</td>
      <td class="num">${libFmtSize(w.bytes)}</td>
      <td>${chips}</td>
      <td>${bag}${map}</td>
      <td class="lib-actions">
        <button data-process="${w.name}" ${busy || running ? "disabled" : ""}
          ${d.can_process ? "" : "disabled"}>${busy ? "running…" : label}</button>
        <button data-map="${w.name}"
          ${busy || running || !w.bag.mcap || !d.can_map ? "disabled" : ""}
          title="${w.bag.mcap ? "rebuild only the scan-matched map from the bag"
                             : "process it first; the map is built from the bag"}"
          >${w.map ? "Rebuild map" : "Build map"}</button>
        <button class="danger" data-forget="${w.name}"
          ${busy ? "disabled" : ""}>Delete</button>
      </td></tr>`;
  }).join("");

  $("lib-table").innerHTML =
    `<tr><th>walk</th><th>when</th><th>length</th><th>size</th>
         <th>checks</th><th>file</th><th></th></tr>` +
    (rows || `<tr><td colspan="7" class="rec-status">no walks here yet</td></tr>`);

  for (const b of $("lib-table").querySelectorAll("[data-process]")) {
    b.onclick = () => libProcess(b.dataset.process);
  }
  for (const b of $("lib-table").querySelectorAll("[data-map]")) {
    b.onclick = () => libMap(b.dataset.map);
  }
  for (const b of $("lib-table").querySelectorAll("[data-forget]")) {
    b.onclick = () => libForget(b.dataset.forget);
  }
}

function libRender(d) {
  const panel = $("panel-library");
  if (!d.available) {
    $("lib-root").textContent = "unavailable";
    $("lib-unavailable").classList.remove("hidden");
    $("lib-unavailable").textContent = d.why || "not available on this machine";
    $("lib-table").classList.add("hidden");
    $("lib-job").classList.add("hidden");
    $("lib-note").textContent = "";
    return;
  }
  $("lib-unavailable").classList.add("hidden");
  $("lib-table").classList.remove("hidden");
  $("lib-root").textContent = d.root;

  libRenderJob(d.job);
  libRenderWalks(d);

  $("lib-note").textContent = d.can_process
    ? "Process makes the bag and the scan-matched map: roughly half the walk's " +
      "length for the bag, and about as long as the walk for the map. Hover a " +
      "check to see what it means."
    : "process_walk.sh or walk_to_bag.sh is not next to dashboard.py, so " +
      "nothing here can be processed.";

  // Poll fast while something is moving, slowly when nothing is.
  const want = d.job && d.job.running ? 1000 : 10000;
  if (want !== libPeriodMs) {
    libPeriodMs = want;
    if (libTimer) clearInterval(libTimer);
    libTimer = setInterval(libRefresh, want);
  }
}

async function libRefresh() {
  if (libBusy) return;                 // a slow disk scan must not stack up
  libBusy = true;
  try {
    const d = await (await fetch("/api/library", { cache: "no-store" })).json();
    libRender(d);
  } catch (e) {
    $("lib-note").textContent = "cannot reach the dashboard server";
  } finally {
    libBusy = false;
  }
}

async function libProcess(name) {
  // Two conversions at once corrupt each other's bag, and re-processing throws
  // away a file that took as long as the walk to make. Both are worth a pause.
  if (!confirm(`Process "${name}"?\n\n` +
      "Makes the bag, then the scan-matched map. For a five-minute walk, " +
      "about two minutes for the bag and six for the map. Any existing bag " +
      "and map for this walk are replaced.")) return;
  try {
    libRender(await libPost("library-process", { name }));
  } catch (e) {
    alert(e.message);
  }
}

async function libMap(name) {
  // Only the map, from the bag already made: about six minutes for a
  // five-minute walk. Replaces the map files, not the bag.
  try {
    libRender(await libPost("library-map", { name }));
  } catch (e) {
    alert(e.message);
  }
}

async function libForget(name) {
  if (!confirm(`Delete "${name}" from this machine?\n\n` +
      "The whole folder goes: the raw recording and any bag built from it. " +
      "If the walk is still on the robot it can be fetched again; if it is " +
      "not, this cannot be undone.")) return;
  try {
    libRender(await libPost("library-forget", { name }));
  } catch (e) {
    alert(e.message);
  }
}

$("btn-job-cancel").onclick = async () => {
  if (!confirm("Stop the conversion?\n\nThe partly written bag is left behind " +
               "and will not be usable.")) return;
  try {
    libRender(await libPost("library-cancel", {}));
  } catch (e) {
    alert(e.message);
  }
};

libRefresh();


// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------
// Depth camera and LiDAR stream switches. Polled every 2 s, every second while
// a start or stop is running. Rates come from status files the relays write
// once a second on the robot.

let senTimer = null;
let senPeriodMs = 0;
let senModesFilled = false;

function senRender(d) {
  if (!d.available) {
    $("sen-unavailable").classList.remove("hidden");
    $("sen-unavailable").textContent = d.why || "not available on this machine";
    $("sen-body").classList.add("hidden");
    $("sen-busy").textContent = "unavailable";
    return;
  }
  $("sen-unavailable").classList.add("hidden");
  $("sen-body").classList.remove("hidden");

  if (!senModesFilled && d.modes) {
    $("sen-mode").innerHTML = d.modes.map((m) =>
      `<option value="${m.id}">${m.label}${m.proven ? "" : " (untested)"}</option>`).join("");
    $("sen-mode").value = d.depth.on && d.depth.mode ? d.depth.mode : d.wanted.depth_mode;
    $("sen-color").checked = d.depth.on ? !!d.depth.color : !!d.wanted.color;
    senModesFilled = true;
  }

  const dep = d.depth;
  const depthOk = dep.on && dep.fps_in > 0;
  $("sen-depth-dot").className = "sen-dot" + (depthOk ? " on" : dep.on ? " warn" : "");
  $("sen-depth-fps").textContent = dep.on
    ? (dep.fps_in == null ? "starting" : `${dep.fps_in.toFixed(1)} fps`)
    : "off";
  let depthNote = dep.note;
  if (dep.on && dep.mode) {
    const [w, h, f] = dep.mode.split("x");
    const cap = dep.note.charAt(0).toUpperCase() + dep.note.slice(1);
    depthNote = `${w} x ${h} at ${f} fps${dep.color ? ", colour on" : ""}. ${cap}.`;
    if (dep.connected && dep.fps != null && dep.fps_in > 0 && dep.fps < dep.fps_in * 0.8) {
      depthNote += ` Sending ${dep.fps.toFixed(1)} fps: the link or laptop is falling behind.`;
    }
  }
  $("sen-depth-note").textContent = depthNote;

  const li = d.lidar;
  const lidarOk = li.on && li.pps > li.pps_expected * 0.6;
  $("sen-lidar-dot").className = "sen-dot" + (lidarOk ? " on" : li.on ? " warn" : "");
  $("sen-lidar-pps").textContent = li.on
    ? (li.pps == null ? "on" : `${Math.round(li.pps)} pkt/s`)
    : "off";
  $("sen-lidar-note").textContent = li.on && li.pps != null
    ? `${li.note}. Expected about ${Math.round(li.pps_expected)} pkt/s.`
    : li.note;
  if (li.dest && document.activeElement !== $("sen-dest")) $("sen-dest").value = li.dest;

  const job = d.job;
  const busy = !!(job && job.running);
  $("sen-busy").textContent = busy ? "busy" : (dep.on || li.on ? "on" : "idle");
  $("sen-busy").className = "rec-mode" + (busy ? " replay" : dep.on || li.on ? " live" : "");
  $("btn-depth-on").disabled = busy;
  $("btn-depth-on").textContent = dep.on ? "Restart with these" : "Start";
  $("btn-depth-off").disabled = busy || !dep.on;
  $("btn-lidar-on").disabled = li.on;
  $("btn-lidar-off").disabled = !li.on;

  const box = $("sen-job");
  if (!job) {
    box.classList.add("hidden");
  } else {
    box.classList.remove("hidden");
    $("sen-job-title").textContent = job.title;
    $("sen-job-elapsed").textContent = `${Math.round(job.elapsed)} s`;
    const bar = $("sen-job-bar");
    bar.style.width = `${job.percent}%`;
    bar.classList.toggle("failed", !job.running && job.returncode !== 0);
    bar.classList.toggle("done", !job.running && job.returncode === 0);
    $("sen-job-log").textContent = (job.log || []).join("\n");
    $("sen-job-log").scrollTop = $("sen-job-log").scrollHeight;
  }

  const want = busy ? 1000 : 2000;
  if (want !== senPeriodMs) {
    senPeriodMs = want;
    if (senTimer) clearInterval(senTimer);
    senTimer = setInterval(senRefresh, want);
  }
}

async function senRefresh() {
  if (!cfg.panels["panel-sensors"]) return;      // hidden: no need to ask
  try {
    senRender(await (await fetch("/api/sensors", { cache: "no-store" })).json());
  } catch (e) {
    $("sen-busy").textContent = "no server";
  }
}

async function senDo(action, params) {
  try {
    senRender(await libPost(action, params));
  } catch (e) {
    alert(e.message);
  }
}

$("btn-depth-on").onclick = () => senDo("sensors-depth-on",
  { mode: $("sen-mode").value, color: $("sen-color").checked });
$("btn-depth-off").onclick = () => senDo("sensors-depth-off", {});
$("btn-lidar-on").onclick = () => senDo("sensors-lidar-on",
  { dest: $("sen-dest").value.trim() || undefined });
$("btn-lidar-off").onclick = () => senDo("sensors-lidar-off", {});

senRefresh();
senPeriodMs = 2000;
senTimer = setInterval(senRefresh, senPeriodMs);


tick();
restartPolling();
showRoute();
setInterval(showRoute, 15000);
loadServerSettings();
refreshPreflight();
setInterval(refreshPreflight, 5000);
