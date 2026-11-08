// Trend graphs -- the last couple of minutes of a few numbers.
//
// Each graph is drawn INSIDE the panel showing the same numbers, directly
// under them: battery under Power, the four foot loads under Foot load, motor
// temperature under the joints table. The point of a graph here is to say
// "and this is where that number has been", so it belongs next to the number,
// not in a separate strip further down the page.
//
// History is kept in the browser, not on the server: the page collects each
// value as it polls /data. That keeps the server stateless and costs the Jetson
// nothing.
//
// It is also written to localStorage, so a reload does not throw away the last
// two minutes. Reloading the page is something you do while working -- after an
// edit, after a network blip -- and losing the trend every time made the graphs
// far less useful than they should be. Samples older than the window are
// discarded on the way back in, so a page opened tomorrow starts empty rather
// than showing yesterday as if it were now.
//
// Drawn on a plain 2D canvas. No charting library -- the Jetson has no
// internet, so every byte would have to be vendored and served.

const WINDOW_SECONDS = 120;
const HEIGHT = 128;              // css pixels of canvas, not counting the label row

// Each graph: where it mounts, what it draws, and how the vertical axis scales.
// `fixed` pins the axis so the shape always means the same thing; without it
// the axis follows the data, which is right for values with no natural range.
export const CHARTS = [
  {
    id: "chart-battery", mount: "slot-power",
    label: "Charge, last 2 min", unit: "%", fixed: [0, 100],
    series: [{ key: "soc", label: "charge", color: "#60a5fa" }],
    sample: (d) => (d.power ? { soc: d.power.soc } : null),
  },
  {
    id: "chart-feet", mount: "slot-feet",
    label: "Foot load, last 2 min", unit: "", fixed: null,
    series: [
      { key: "FR", label: "FR", color: "#60a5fa" },
      { key: "FL", label: "FL", color: "#4ade80" },
      { key: "RR", label: "RR", color: "#fbbf24" },
      { key: "RL", label: "RL", color: "#f87171" },
    ],
    sample: (d) => {
      if (!d.feet) return null;
      const o = {};
      for (const f of d.feet) o[f.name] = f.force;
      return o;
    },
  },
  {
    id: "chart-motors", mount: "slot-joints",
    label: "Motor temperature, last 2 min", unit: "°C", fixed: [20, 90],
    series: [
      { key: "max", label: "hottest", color: "#f87171" },
      { key: "avg", label: "average", color: "#60a5fa" },
    ],
    sample: (d) => {
      if (!d.joints) return null;
      const t = d.joints.map((j) => j.temp);
      return { max: Math.max(...t), avg: t.reduce((a, b) => a + b, 0) / t.length };
    },
  },
];

const STORE_KEY = "go2.charts";
const SAVE_EVERY_MS = 5000;      // often enough to survive a reload, rare
                                 // enough not to serialise on every frame

export class Charts {
  constructor() {
    this.history = loadHistory();
    this.canvases = {};
    this.lastSaved = 0;

    for (const c of CHARTS) {
      const slot = document.getElementById(c.mount);
      if (!slot) continue;       // the panel it belongs to is not on this page
      const wrap = document.createElement("div");
      wrap.className = "chart";
      wrap.id = c.id;
      wrap.innerHTML =
        `<div class="chart-head"><span>${c.label}</span>` +
        `<span class="chart-keys">` +
        c.series.map((s) =>
          `<span><i style="background:${s.color}"></i>${s.label}</span>`).join("") +
        `</span><span class="chart-now" data-now></span></div>` +
        `<canvas></canvas>`;
      slot.appendChild(wrap);
      this.canvases[c.id] = wrap.querySelector("canvas");
    }

    // Panels resize when the window does, and a canvas does not reflow on its
    // own, so redraw whenever the layout changes.
    if (window.ResizeObserver) {
      const ro = new ResizeObserver(() => this.draw());
      for (const c of CHARTS) {
        const slot = document.getElementById(c.mount);
        if (slot) ro.observe(slot);
      }
    }
  }

  /** Called on every poll. Stores one sample and redraws. */
  push(d) {
    const now = Date.now() / 1000;
    const values = {};
    for (const c of CHARTS) {
      const v = c.sample(d);
      if (v) values[c.id] = v;
    }
    if (!Object.keys(values).length) return;

    this.history.push({ t: now, values });
    // Drop anything older than the window so memory stays flat however long
    // the page is left open.
    const cutoff = now - WINDOW_SECONDS;
    while (this.history.length && this.history[0].t < cutoff) this.history.shift();

    // Throttled, and wrapped: localStorage throws when it is full or disabled,
    // and a graph is not worth taking the page down for.
    if (Date.now() - this.lastSaved > SAVE_EVERY_MS) {
      this.lastSaved = Date.now();
      try {
        localStorage.setItem(STORE_KEY, JSON.stringify(this.history));
      } catch (e) { /* full, or private browsing */ }
    }

    this.draw();
  }

  draw() {
    const css = getComputedStyle(document.documentElement);
    const line = css.getPropertyValue("--line").trim() || "#252a34";
    const faint = css.getPropertyValue("--faint").trim() || "#5b6472";
    const now = Date.now() / 1000;

    for (const c of CHARTS) {
      const wrap = document.getElementById(c.id);
      if (!wrap || wrap.classList.contains("hidden")) continue;
      if (!wrap.offsetParent) continue;      // the whole panel is hidden

      const cv = this.canvases[c.id];
      const w = cv.clientWidth;
      if (!w) continue;
      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(HEIGHT * dpr)) {
        cv.width = Math.round(w * dpr);
        cv.height = Math.round(HEIGHT * dpr);
      }
      const ctx = cv.getContext("2d");
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, HEIGHT);

      const pts = this.history.filter((s) => s.values[c.id]);

      // Leave room on the left for the axis numbers.
      const padL = 34, padT = 6, padB = 15;
      const plotW = w - padL - 4;
      const plotH = HEIGHT - padT - padB;

      if (pts.length < 2) {
        ctx.fillStyle = faint;
        ctx.font = "11px system-ui, sans-serif";
        ctx.fillText("collecting…", padL, HEIGHT / 2);
        continue;
      }

      // --- vertical range ---
      let lo, hi;
      if (c.fixed) {
        [lo, hi] = c.fixed;
      } else {
        lo = Infinity; hi = -Infinity;
        for (const p of pts) {
          for (const s of c.series) {
            const v = p.values[c.id][s.key];
            if (v === undefined) continue;
            if (v < lo) lo = v;
            if (v > hi) hi = v;
          }
        }
        if (!isFinite(lo)) continue;
        const pad = (hi - lo) * 0.15 || 1;
        lo = Math.floor(lo - pad); hi = Math.ceil(hi + pad);
      }
      const span = hi - lo || 1;

      // Time axis. Until two minutes have been collected the axis covers only
      // what we actually have, so the trace fills the width from the start
      // instead of huddling against the right edge. Once the window is full it
      // stops growing and the trace scrolls.
      const oldest = pts[0].t;
      const covered = Math.min(WINDOW_SECONDS, Math.max(now - oldest, 10));
      const x = (t) => padL + ((t - (now - covered)) / covered) * plotW;
      const y = (v) => padT + plotH - ((v - lo) / span) * plotH;

      // --- gridlines and axis labels ---
      ctx.strokeStyle = line;
      ctx.lineWidth = 1;
      ctx.fillStyle = faint;
      ctx.font = "10px system-ui, sans-serif";
      ctx.textAlign = "right";
      for (let i = 0; i <= 2; i++) {
        const v = lo + (span * i) / 2;
        const py = Math.round(y(v)) + 0.5;
        ctx.beginPath();
        ctx.moveTo(padL, py); ctx.lineTo(w - 4, py);
        ctx.stroke();
        ctx.fillText(v.toFixed(0) + c.unit, padL - 5, py + 3);
      }
      ctx.textAlign = "left";
      const ago = covered >= WINDOW_SECONDS
        ? "2 min ago"
        : covered >= 60
          ? Math.round(covered / 60) + " min ago"
          : Math.round(covered) + "s ago";
      ctx.fillText(ago, padL, HEIGHT - 4);
      ctx.textAlign = "right";
      ctx.fillText("now", w - 4, HEIGHT - 4);
      ctx.textAlign = "left";

      // --- the traces ---
      for (const s of c.series) {
        ctx.strokeStyle = s.color;
        ctx.lineWidth = 1.6;
        ctx.beginPath();
        let started = false;
        for (const p of pts) {
          const v = p.values[c.id][s.key];
          if (v === undefined) continue;
          const px = x(p.t), py = y(v);
          if (started) ctx.lineTo(px, py);
          else { ctx.moveTo(px, py); started = true; }
        }
        ctx.stroke();
      }

      // --- the newest value, in the label row ---
      const latest = pts[pts.length - 1].values[c.id];
      const nowEl = wrap.querySelector("[data-now]");
      if (nowEl) {
        nowEl.textContent = c.series
          .filter((s) => latest[s.key] !== undefined)
          .map((s) => latest[s.key].toFixed(0))
          .join(" / ") + (c.unit ? " " + c.unit : "");
      }
    }
  }

  /** Show or hide one graph. */
  setVisible(id, visible) {
    const el = document.getElementById(id);
    if (el) el.classList.toggle("hidden", !visible);
    this.draw();
  }

  clear() {
    this.history = [];
    // Also from storage, or the trend would reappear on the next reload and
    // "Clear graph history" would look like it had done nothing.
    try { localStorage.removeItem(STORE_KEY); } catch (e) {}
    this.draw();
  }
}


/** The stored history, minus anything already outside the window. */
function loadHistory() {
  let raw;
  try {
    raw = JSON.parse(localStorage.getItem(STORE_KEY) || "[]");
  } catch (e) {
    return [];
  }
  if (!Array.isArray(raw)) return [];
  const cutoff = Date.now() / 1000 - WINDOW_SECONDS;
  return raw.filter((s) => s && typeof s.t === "number" && s.t >= cutoff);
}
