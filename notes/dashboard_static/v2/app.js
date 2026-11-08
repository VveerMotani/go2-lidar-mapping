// Dashboard v2: the redesigned robot page, served at /dashboard-v2.
//
// Same API as the original page (notes/dashboard.py), nothing new to trust:
//   /data             robot state, 5 times a second
//   /api/preflight    the checks before a walk
//   /api/recordings   walk status, last report, walks on the robot
//   /api/sensors      depth camera, LiDAR relay, cameras
//   /api/system       this computer (v2 only)
//   /api/control      every button: walk-start/stop/delete, set-clock,
//                     sensors-*, sport (v2), v2-layout (v2)
// The approved design is notes/dashboard_static/mockup.html.

import { Robot3D } from "/static/robot3d.js";
import * as THREE from "/static/vendor/three.module.js";

const $ = s => document.querySelector(s), $$ = s => [...document.querySelectorAll(s)];
const E = id => document.getElementById(id);
const esc = v => String(v ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const fmtB = b => b == null ? '-' : b >= 1e9 ? (b / 1e9).toFixed(1) + ' GB' : b >= 1e6 ? Math.round(b / 1e6) + ' MB' : Math.round(b / 1e3) + ' kB';
const clock = s => String(Math.floor((s || 0) / 60)).padStart(2, '0') + ':' + String(Math.floor(s || 0) % 60).padStart(2, '0');
const RAMP = [[0, [59, 130, 246]], [.5, [74, 222, 128]], [.75, [251, 191, 36]], [1, [248, 113, 113]]];
function ramp(t) {
  t = Math.max(0, Math.min(1, t));
  for (let i = 1; i < RAMP.length; i++) if (t <= RAMP[i][0]) {
    const [a, ca] = RAMP[i - 1], [b, cb] = RAMP[i], f = (t - a) / (b - a);
    return `rgb(${ca.map((c, k) => Math.round(c + (cb[k] - c) * f)).join(',')})`;
  }
  return 'rgb(248,113,113)';
}
let toastH;
function toast(t, bad) {
  const el = E('toast'); el.textContent = t; el.hidden = false;
  el.style.borderLeftColor = bad ? 'var(--bad)' : 'var(--accent)';
  clearTimeout(toastH); toastH = setTimeout(() => el.hidden = true, bad ? 6000 : 3000);
}

/* ------------------------------------------------------------------ API */
async function get(path, timeout = 4000) {
  const c = new AbortController(), t = setTimeout(() => c.abort(), timeout);
  try {
    const r = await fetch(path, { cache: 'no-store', signal: c.signal });
    if (!r.ok) throw new Error(path + ' ' + r.status);
    return await r.json();
  } finally { clearTimeout(t); }
}
async function control(action, extra) {
  const r = await fetch('/api/control', { method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(Object.assign({ action }, extra || {})) });
  let j = {};
  try { j = await r.json(); } catch (e) { /* not JSON */ }
  if (!r.ok) { const err = new Error(j.error || ('HTTP ' + r.status)); err.body = j; err.status = r.status; throw err; }
  return j;
}

/* ---------------------------------------------------------------- state */
const S = { d: null, pf: null, rec: null, sens: null, sys: null, settings: null, fails: 0, online: true, lastOk: 0 };
const H = { t: [], soc: [], hot: [], avg: [], feet: [[], [], [], []] };        // chart history
const SOC = [];                                                                 // for time left
let trail = [], trailFrom = 'page opened', wasRecording = false;
E('host').textContent = location.host;
// Inside Go2 Studio (?embed=studio) Studio records and downloads walks itself,
// so the page leaves those to it rather than offering a second way.
const EMBED = new URLSearchParams(location.search).has('embed');
if (EMBED) document.body.classList.add('embed');

/* ------------------------------------------------------------------ nav */
$$('.rail button[data-p]').forEach(b => b.onclick = () => {
  $$('.rail button[data-p]').forEach(x => x.removeAttribute('aria-current'));
  b.setAttribute('aria-current', 'page');
  $$('.page').forEach(p => p.hidden = p.id !== 'p-' + b.dataset.p);
  scrollTo(0, 0);
  if (b.dataset.p === 'system') { loadSystem(); loadLog(); }
  if (b.dataset.p === 'record') paintRecord();
});

/* --------------------------------------------------------------- gauges */
function arcPath(cx, cy, r, a0, a1) {
  const p = a => [cx + r * Math.cos(a), cy + r * Math.sin(a)];
  const [x0, y0] = p(a0), [x1, y1] = p(a1);
  return `M${x0} ${y0} A${r} ${r} 0 ${a1 - a0 > Math.PI ? 1 : 0} 1 ${x1} ${y1}`;
}
function gauge(el, { value, min, max, unit, sub, color, lo, hi, dec = 0, bands }) {
  // A 270-degree instrument dial: thin track, tick scale, white pointer.
  if (!el) return;
  if (value == null || isNaN(value)) { el.innerHTML = `<div class="empty"><b>-</b>${esc(sub || 'no data')}</div>`; return; }
  const A0 = Math.PI * 3 / 4, A1 = Math.PI * 9 / 4, cx = 88, cy = 80, r = 62;
  const ang = v => A0 + (A1 - A0) * Math.max(0, Math.min(1, (v - min) / (max - min)));
  let t = '';
  for (let i = 0; i <= 40; i++) {
    const a = A0 + (A1 - A0) * i / 40, major = i % 10 === 0, r1 = r + 6, r2 = r + (major ? 15 : 10);
    t += `<line x1="${cx + r1 * Math.cos(a)}" y1="${cy + r1 * Math.sin(a)}" x2="${cx + r2 * Math.cos(a)}" y2="${cy + r2 * Math.sin(a)}" stroke="${major ? '#8a8e92' : '#3a3d41'}" stroke-width="${major ? 1.6 : 1}"/>`;
    if (major) { const v = min + (max - min) * i / 40, rr = r + 25; t += `<text x="${cx + rr * Math.cos(a)}" y="${cy + rr * Math.sin(a) + 3.5}" text-anchor="middle" fill="#7d8185" font-size="10" font-family="Bahnschrift" font-weight="500">${+v.toFixed(max - min < 5 ? 1 : 0)}</text>`; }
  }
  const band = (from, to, c) => `<path d="${arcPath(cx, cy, r, ang(from), ang(to))}" stroke="${c}" stroke-width="6" fill="none" opacity=".55"/>`;
  const inBand = bands ? (bands.find(b => value >= b[0] && value <= b[1]) || bands[0])[2] : null;
  const status = inBand || (color === '#f2c200' || color === '#ff453a' ? color : null);
  const bandsSvg = bands ? bands.map(([a, b, c]) => `<path d="${arcPath(cx, cy, r + 4, ang(a) + .01, ang(b) - .01)}" stroke="${c}" stroke-width="2" fill="none" opacity=".75"/>`).join('') : '';
  const pa = ang(value);
  el.innerHTML = `<svg viewBox="0 0 176 146" style="overflow:visible"><path d="${arcPath(cx, cy, r, A0, A1)}" stroke="#26282b" stroke-width="6" fill="none"/>
      ${bandsSvg}${lo != null ? band(min, lo, '#ff453a') : ''}${hi != null ? band(hi, max, '#ff453a') : ''}
      <path d="${arcPath(cx, cy, r, A0, Math.max(A0 + .001, pa))}" stroke="${status || color || '#f2f2ef'}" stroke-width="6" fill="none"/>
      ${t}<line x1="${cx + (r - 12) * Math.cos(pa)}" y1="${cy + (r - 12) * Math.sin(pa)}" x2="${cx + (r + 8) * Math.cos(pa)}" y2="${cy + (r + 8) * Math.sin(pa)}" stroke="#ffffff" stroke-width="2.5"/></svg>
    <div class="val live-val"><b style="${status ? 'color:' + status : ''}">${value.toFixed(dec)}<small>${unit}</small></b><span>${esc(sub || '')}</span></div>`;
}
const BATT_BANDS = [[0, 25, '#ff453a'], [25, 50, '#ff8a1f'], [50, 75, '#f2c200'], [75, 100, '#3ccf6e']];
const LEGS = { FR: [0, 0], FL: [3, 1], RR: [6, 2], RL: [9, 3] };
const LEGNAME = { FR: 'Front right', FL: 'Front left', RR: 'Rear right', RL: 'Rear left' };

/* ------------------------------------------------- persistent parts (3D, charts) */
const heroWrap = document.createElement('div');
heroWrap.className = 'hero-wrap';
heroWrap.innerHTML = `<canvas id="robot"></canvas><div id="tooltip"></div>
  <div class="hud-top"><div class="seg"><button aria-pressed="true" id="legs-btn" title="Show or hide the four leg cards"><svg class="i"><use href="#i-eye"/></svg>Legs</button></div>
    <div class="seg" id="colormode"><button aria-pressed="true" data-c="temperature">Temperature</button><button data-c="torque">Torque</button><button data-c="off">Plain</button></div>
    <div class="seg"><button aria-pressed="true" data-v="iso" title="3D"><svg class="i"><use href="#i-cube"/></svg></button><button data-v="top" title="Top"><svg class="i"><use href="#i-top"/></svg></button><button data-v="side" title="Side"><svg class="i"><use href="#i-side"/></svg></button></div></div>
  <div class="corner fl" id="c-FL"></div><div class="corner fr" id="c-FR"></div><div class="corner rl" id="c-RL"></div><div class="corner rr" id="c-RR"></div>
  <div class="legend"><span>20 °C</span><span class="ramp"></span><span>80 °C</span><span class="t3">·</span><span>rings: share of weight</span></div>`;
const PARK = 'position:fixed; left:-9999px; top:0; width:600px; height:500px';
heroWrap.style.cssText = PARK;
document.body.appendChild(heroWrap);
const CH = { batt: document.createElement('div'), temp: document.createElement('div'), feet: document.createElement('div') };
Object.values(CH).forEach(w => { w.className = 'chart-wrap'; w.style.cssText = 'position:fixed; left:-9999px; width:300px; height:150px'; document.body.appendChild(w); });

/* ------------------------------------------------------------ actions */
const ACTIONS = [
  { name: 'Posture', items: [['i-stand', 'StandUp', 'Stand up', 1004], ['i-lie', 'StandDown', 'Lie down', 1005], ['i-balance', 'BalanceStand', 'Balance stand', 1002],
    ['i-sit', 'Sit', 'Sit', 1009], ['i-stand', 'RiseSit', 'Rise from sit', 1010], ['i-recover', 'RecoveryStand', 'Recover', 1006]] },
  { name: 'Tricks', items: [['i-wave', 'Hello', 'Wave', 1016], ['i-stretch', 'Stretch', 'Stretch', 1017], ['i-hips', 'Content', 'Happy', 1020],
    ['i-heart', 'Heart', 'Heart', 1036], ['i-dance', 'Dance1', 'Dance 1', 1022], ['i-dance', 'Dance2', 'Dance 2', 1023], ['i-scrape', 'Scrape', 'Scrape', 1029]] },
  { name: 'Locked while the LiDAR is mounted', locked: true, items: [['i-flip', 'FrontFlip', 'Front flip', 1030], ['i-jump', 'FrontJump', 'Front jump', 1031], ['i-jump', 'FrontPounce', 'Pounce', 1032]] },
];
const WHAT = { Hello: 'lifts a front leg and waves', Stretch: 'leans back and stretches', Content: 'wiggles happily', Heart: 'draws a heart with a front leg',
  Dance1: 'dances for about 10 s', Dance2: 'dances for about 10 s', Scrape: 'scrapes the floor with a front foot', StandUp: 'stands up from lying',
  StandDown: 'lies down slowly', BalanceStand: 'stands and holds its balance', Sit: 'sits on its rear legs', RiseSit: 'gets up from sitting', RecoveryStand: 'rights itself and stands' };
const tileHTML = ([ic, api, label, id], locked) => `<button class="tile ${locked ? 'locked' : ''}" ${locked ? 'disabled' : `data-move="${api}"`} title="${api} (${id})">
  ${locked ? '<span class="lock">LOCKED</span>' : ''}<svg class="i"><use href="#${ic}"/></svg><b>${label}</b><span>${api} · ${id}</span></button>`;
E('action-groups').innerHTML = ACTIONS.map(g => `<div class="agroup"><h3>${g.name}</h3><div class="tiles">${g.items.map(a => tileHTML(a, g.locked)).join('')}</div></div>`).join('');

let confirmFn = null;
function ask(title, text, go, fn) {
  E('m-title').textContent = title; E('m-text').textContent = text; E('m-go').textContent = go || 'Do it';
  confirmFn = fn; E('mscrim').hidden = false; E('modal').hidden = false;
}
function closeModal() { E('mscrim').hidden = true; E('modal').hidden = true; confirmFn = null; }
E('m-go').onclick = () => { const f = confirmFn; closeModal(); if (f) f(); };
E('mscrim').onclick = closeModal;
async function sport(move) {
  const tiles = $$(`[data-move="${move}"]`);
  tiles.forEach(t => t.classList.add('busy'));
  try {
    await control('sport', { move });
    toast(move === 'StopMove' ? 'Stopped: the robot holds still.' : 'Sent ' + move + '.');
  } catch (e) { toast(e.message, true); }
  setTimeout(() => tiles.forEach(t => t.classList.remove('busy')), 2400);
}
function askMove(api) {
  const a = ACTIONS.flatMap(g => g.items).find(x => x[1] === api);
  if (recording()) { toast('Moves are paused while a walk records.', true); return; }
  ask(a[2] + '?', 'The robot ' + (WHAT[api] || 'moves') + '. Clear about 1 m around it and keep the remote in hand.', 'Do it', () => sport(api));
}

/* ------------------------------------------------------------ cards */
const CARDS = {
  battery: { title: 'Battery', icon: 'i-batt', what: 'Charge, voltage, current, time left', w: 3, h: 6, minW: 2, minH: 4,
    body: () => `<div class="gauge" id="g-batt"></div><div style="padding:0 12px"><div class="stat-row"><span>Voltage</span><b id="volts">-</b></div><div class="stat-row"><span>Current</span><b id="amps">-</b></div><div class="stat-row"><span>Time left</span><b id="tleft">-</b></div></div>` },
  temp: { title: 'Hottest motor', icon: 'i-heat', what: 'The hottest of the 12 motors', w: 3, h: 5, minW: 2, minH: 4, center: true, body: () => `<div class="gauge" id="g-temp"></div>` },
  speed: { title: 'Speed', icon: 'i-gauge', what: 'Speed, body height, gait', w: 3, h: 6, minW: 2, minH: 4,
    body: () => `<div class="gauge" id="g-speed"></div><div style="padding:0 12px"><div class="stat-row"><span>Body height</span><b id="height">-</b></div><div class="stat-row"><span>Gait</span><b id="gait">-</b></div></div>` },
  robot: { title: 'Live model', icon: 'i-cube', what: 'The 3D Go2 with temperatures and foot load', w: 6, h: 12, minW: 4, minH: 6, mount: bd => { bd.appendChild(heroWrap); heroWrap.style.cssText = ''; } },
  sensors: { title: 'Sensors', icon: 'i-sens', what: 'Every stream; open the LiDAR row for its settings', w: 3, h: 6, minW: 3, minH: 4, body: () => `<div id="sys-list" style="position:absolute; inset:0; overflow:auto; padding:0 12px"></div>` },
  trail: { title: 'Trail', icon: 'i-map', what: 'Where the robot has walked, and how far from the start', w: 3, h: 6, minW: 3, minH: 4,
    extra: '<button class="btn ghost sm" data-a="trail-reset" title="Start the trail here">Reset</button>',
    body: () => `<svg class="mini-map" id="trail" viewBox="0 0 300 260" preserveAspectRatio="xMidYMid meet"></svg><div class="map-read"><b id="trail-d">-</b><span>from the start</span></div><div class="map-foot"><span id="trail-l"></span><span id="trail-g"></span></div>` },
  attitude: { title: 'Attitude', icon: 'i-att', what: 'Roll and pitch, like an aircraft horizon', w: 3, h: 4, minW: 2, minH: 3, center: true, extra: '<span id="att-t" class="t2" style="font:600 12px var(--num)"></span>', body: () => `<svg class="att" id="att" viewBox="0 0 200 200"></svg>` },
  camera: { title: 'Camera', icon: 'i-cam', what: 'Live picture from a robot camera', w: 3, h: 5, minW: 2, minH: 3,
    extra: '<div class="seg" id="camsel"><button aria-pressed="true" data-cam="2">2</button><button data-cam="4">4</button></div>',
    body: () => `<div class="camimg"><img id="camimg" alt="camera"><div class="none" id="camnone" hidden></div></div>` },
  feet: { title: 'Foot load', icon: 'i-feet', what: 'Top view: each foot sized and coloured by its share', w: 3, h: 5, minW: 2, minH: 4, body: () => `<svg class="feetview" id="feetview" viewBox="0 0 200 220"></svg>` },
  'ch-batt': { title: 'Battery, 2 min', icon: 'i-chart', what: 'Charge over the last two minutes', w: 3, h: 5, minW: 2, minH: 3, extra: '<span class="big-now" id="now-batt"></span>', mount: bd => mountChart(bd, 'batt') },
  'ch-temp': { title: 'Motor temperature', icon: 'i-chart', what: 'Hottest and average motor', w: 3, h: 5, minW: 2, minH: 3, extra: '<span class="keys"><span><i style="background:#ff6a1a"></i>hottest</span><span><i style="background:#8a8e92"></i>average</span></span>', mount: bd => mountChart(bd, 'temp') },
  'ch-feet': { title: 'Foot load, 2 min', icon: 'i-chart', what: 'The four feet over the last two minutes, in newtons', w: 3, h: 5, minW: 2, minH: 3, extra: '<span class="keys"><span><i style="background:#5aa9ff"></i>FL</span><span><i style="background:#3ccf6e"></i>FR</span><span><i style="background:#f2c200"></i>RL</span><span><i style="background:#ff6a1a"></i>RR</span></span>', mount: bd => mountChart(bd, 'feet') },
  heat: { title: 'Motor temperatures', icon: 'i-heat', what: 'All 12 motors as a colour grid', w: 4, h: 5, minW: 3, minH: 4, body: () => `<div class="heat" id="heat" style="padding:0 12px"></div>` },
  joints: { title: 'Joints', icon: 'i-list', what: 'Angle, torque and temperature of every joint', w: 4, h: 9, minW: 3, minH: 5, body: () => `<div style="position:absolute; inset:0; overflow:auto; padding:0 6px"><table class="walks" id="jtab"></table></div>` },
  quickrec: { title: 'Record a walk', icon: 'i-rec', what: 'Checks, start and stop, without leaving Live', w: 3, h: 6, minW: 3, minH: 5, body: () => `<div id="qr" style="padding:0 12px"></div>` },
  depth: { title: 'Depth camera', icon: 'i-cam', what: 'Frame rate and on/off', w: 3, h: 5, minW: 2, minH: 4, center: true, body: () => `<div><div class="gauge" id="g-depth"></div><div style="text-align:center; margin-top:4px" id="depth-btn"></div></div>` },
  log: { title: 'Robot log', icon: 'i-term', what: 'Last lines of the dashboard log', w: 6, h: 4, minW: 3, minH: 3, body: () => `<pre class="log" id="cardlog" style="position:absolute; inset:0; margin:0; max-height:none"></pre>` },
  walks: { title: 'Walks on the robot', icon: 'i-down', what: 'Recorded walks with Download', w: 6, h: 5, minW: 4, minH: 4, body: () => `<div style="position:absolute; inset:0; overflow:auto; padding:0 6px"><table class="walks" id="cardwalks"></table></div>` },
  actions: { title: 'Actions', icon: 'i-paw', what: 'Stand, sit, wave and the other moves', w: 3, h: 6, minW: 3, minH: 5,
    body: () => `<div style="padding:0 12px"><div class="minitiles">${[ACTIONS[0].items[0], ACTIONS[0].items[1], ACTIONS[0].items[3], ...ACTIONS[1].items.slice(0, 3)].map(a => tileHTML(a)).join('')}</div>
      <button class="btn danger" style="width:100%; justify-content:center; margin-top:8px" data-act="stop"><svg class="i"><use href="#i-stop"/></svg>STOP</button>
      <button class="btn ghost sm" style="width:100%; justify-content:center; margin-top:4px" data-goto="actions">All actions</button></div>` },
};
const PRESETS = {
  Mission: { what: 'Robot in the middle, gauges and the trail round it',
    items: [['battery', 0, 0, 3, 6], ['temp', 0, 6, 3, 6], ['robot', 3, 0, 6, 12], ['sensors', 9, 0, 3, 6], ['trail', 9, 6, 3, 6],
      ['camera', 0, 12, 3, 5], ['ch-batt', 3, 12, 3, 5], ['ch-temp', 6, 12, 3, 5], ['ch-feet', 9, 12, 3, 5]] },
  Recording: { what: 'For walking the robot: record, trail, camera',
    items: [['quickrec', 0, 0, 3, 7], ['robot', 3, 0, 6, 10], ['trail', 9, 0, 3, 5], ['camera', 9, 5, 3, 5], ['battery', 0, 7, 3, 6], ['sensors', 3, 10, 5, 4], ['walks', 8, 10, 4, 5], ['actions', 0, 13, 3, 6]] },
  Engineering: { what: 'Motors, joints, charts and the log',
    items: [['robot', 0, 0, 5, 10], ['heat', 5, 0, 4, 5], ['joints', 9, 0, 3, 10], ['ch-temp', 5, 5, 4, 5], ['ch-feet', 0, 10, 5, 5], ['log', 5, 10, 7, 5]] },
};

/* ------------------------------------------------------------ grid */
const grid = GridStack.init({ column: 12, cellHeight: 60, margin: 0, float: false, animate: true, handle: '.hd',
  disableDrag: true, disableResize: true, resizable: { handles: 'se' } }, '#live-grid');
const wide = () => innerWidth > 900;
let loading = false, layoutName = 'Mission', saveT = null;
function cardEl(id, x, y, w, h) {
  const c = CARDS[id], el = document.createElement('div');
  el.className = 'grid-stack-item';
  el.setAttribute('gs-id', id);
  [['x', x], ['y', y], ['w', w ?? c.w], ['h', h ?? c.h], ['min-w', c.minW], ['min-h', c.minH]].forEach(([k, v]) => v != null && el.setAttribute('gs-' + k, v));
  el.style.setProperty('--h', h ?? c.h);
  el.style.order = id === 'robot' ? -1 : (y ?? 99) * 12 + (x ?? 0);
  el.innerHTML = `<div class="grid-stack-item-content"><div class="card">
    <div class="hd"><svg class="i grip" style="width:16px;height:16px"><use href="#i-grip"/></svg><svg class="i t3" style="width:16px;height:16px"><use href="#${c.icon}"/></svg><span class="cap">${c.title}</span>
      <span class="r">${c.extra || ''}<button class="x" data-rm="${id}" title="Remove this card"><svg class="i"><use href="#i-x"/></svg></button></span></div>
    <div class="bd ${c.center ? 'center' : ''}">${c.body ? c.body() : ''}</div></div></div>`;
  return el;
}
function park(id) {
  if (id === 'robot') { document.body.appendChild(heroWrap); heroWrap.style.cssText = PARK; }
  const ch = { 'ch-batt': 'batt', 'ch-temp': 'temp', 'ch-feet': 'feet' }[id];
  if (ch) { document.body.appendChild(CH[ch]); CH[ch].style.cssText = 'position:fixed; left:-9999px; width:300px; height:150px'; }
}
function addCard(id, x, y, w, h) {
  const el = cardEl(id, x, y, w, h);
  E('live-grid').appendChild(el);
  grid.makeWidget(el);
  if (CARDS[id].mount) CARDS[id].mount(el.querySelector('.bd'));
  paintAll();
  if (id === 'camera') setCamera();
  if (id === 'log') loadCardLog();
}
function applyLayout(items, name) {
  loading = true;
  grid.float(true);
  grid.getGridItems().forEach(el => { park(el.getAttribute('gs-id')); grid.removeWidget(el, true, false); });
  [...items].sort((a, b) => a[2] - b[2] || a[1] - b[1]).forEach(([id, x, y, w, h]) => CARDS[id] && addCard(id, x, y, w, h));
  grid.float(false);
  loading = false;
  layoutName = name || 'Custom';
  E('layout-name').textContent = 'Layout: ' + layoutName;
}
function saveLayout() {
  if (!wide()) return;
  const layout = { name: layoutName, items: grid.engine.nodes.map(n => [n.id, n.x, n.y, n.w, n.h]) };
  try { localStorage.setItem('go2.v2.layout', JSON.stringify(layout)); } catch (e) { /* private window */ }
  clearTimeout(saveT);
  saveT = setTimeout(() => control('v2-layout', { layout }).catch(e => toast('Layout kept on this device only: ' + e.message, true)), 800);
}
grid.on('change', () => { if (loading) return; layoutName = 'Custom'; E('layout-name').textContent = 'Layout: Custom'; saveLayout(); });
function setEdit(on) { document.body.classList.toggle('edit', on); grid.enableMove(on); grid.enableResize(on); }
function catalog() {
  const have = new Set(grid.getGridItems().map(el => el.getAttribute('gs-id')));
  E('catalog').innerHTML = Object.entries(CARDS).map(([id, c]) => `<div class="cat ${have.has(id) ? 'on' : ''}"><div class="ic"><svg class="i"><use href="#${c.icon}"/></svg></div>
    <div><b>${c.title}</b><span>${c.what}</span></div>
    ${have.has(id) ? `<button class="btn ghost sm" data-rm="${id}">Remove</button>` : `<button class="btn sm" data-add="${id}"><svg class="i"><use href="#i-plus"/></svg>Add</button>`}</div>`).join('');
  E('presets').innerHTML = Object.entries(PRESETS).map(([n, p]) => `<button class="preset" data-preset="${n}">${miniMap(p.items)}<b>${n}</b><span>${p.what}</span></button>`).join('');
}
function miniMap(items) {
  const H2 = Math.max(...items.map(i => i[2] + i[4]));
  return `<svg viewBox="0 0 120 ${H2 * 3}" preserveAspectRatio="none">${items.map(([id, x, y, w, h]) => `<rect x="${x * 10 + .8}" y="${y * 3 + .8}" width="${w * 10 - 1.6}" height="${h * 3 - 1.6}" rx="1" fill="${id === 'robot' ? '#ff6a1a' : '#2e3033'}"/>`).join('')}</svg>`;
}
function closeDrawer() { E('scrim').hidden = true; E('drawer').hidden = true; }
E('scrim').onclick = closeDrawer;

/* ------------------------------------------------------------ 3D */
const robot = new Robot3D(heroWrap.querySelector('#robot'), heroWrap.querySelector('#tooltip'));
robot.load().then(() => {
  robot.scene.children.filter(o => o.type === 'GridHelper').forEach(g => robot.scene.remove(g));
  const polar = new THREE.PolarGridHelper(1.1, 12, 5, 96, 0x44474b, 0x242629);
  polar.rotation.x = Math.PI / 2;
  robot.scene.add(polar);
  iso();
}).catch(e => console.error('3D model', e));
const iso = () => robot._moveTo(1.05, -1.05, 0.62);
robot.resetView = iso;
const legsBtn = heroWrap.querySelector('#legs-btn');
const setLegs = on => { heroWrap.classList.toggle('nolegs', !on); legsBtn.setAttribute('aria-pressed', on); try { localStorage.setItem('go2.v2.legs', on ? '1' : '0'); } catch (e) { /* private */ } };
legsBtn.onclick = () => setLegs(heroWrap.classList.contains('nolegs'));
try { setLegs(localStorage.getItem('go2.v2.legs') !== '0'); } catch (e) { setLegs(true); }
heroWrap.querySelectorAll('#colormode button').forEach(b => b.onclick = () => { heroWrap.querySelectorAll('#colormode button').forEach(x => x.setAttribute('aria-pressed', x === b)); robot.setColorMode(b.dataset.c); });
heroWrap.querySelectorAll('[data-v]').forEach(b => b.onclick = () => { heroWrap.querySelectorAll('[data-v]').forEach(x => x.setAttribute('aria-pressed', x === b)); ({ iso, top: () => robot.topView(), side: () => robot.sideView() })[b.dataset.v](); });

/* ------------------------------------------------------------ charts */
const now = () => Date.now() / 1000;
const hms = s => { const d = new Date(s * 1000), p = n => String(n).padStart(2, '0'); return p(d.getHours()) + ':' + p(d.getMinutes()) + ':' + p(d.getSeconds()); };
// A foot's load in newtons, worked out on the robot from its leg motors; the raw
// foot sensors disagree with each other (see leg_load in dashboard.py).
const load = f => f.load != null ? f.load : Math.max(0, f.force);
const axis = { stroke: '#7d8185', grid: { stroke: 'rgba(255,255,255,.05)', width: 1 }, ticks: { show: false }, font: '600 11px Bahnschrift, sans-serif' };
const line = (c, w = 2) => ({ stroke: c, width: w, points: { show: false } });
function mk(wrap, series, data) {
  return new uPlot({ width: 300, height: 150, cursor: { show: false }, legend: { show: false },
    scales: { x: { time: true } },
    axes: [{ ...axis, space: 70, values: (u, v) => v.map(hms) }, { ...axis, size: 36 }],
    series: [{}, ...series] }, data, wrap);
}
const PLOT = {
  batt: mk(CH.batt, [{ ...line('#f2f2ef', 1.8), fill: 'rgba(255,255,255,.05)' }], [[], []]),
  temp: mk(CH.temp, [line('#ff6a1a'), line('#8a8e92')], [[], [], []]),
  feet: mk(CH.feet, ['#5aa9ff', '#3ccf6e', '#f2c200', '#ff6a1a'].map(c => line(c, 1.5)), [[], [], [], [], []]),
};
function fit(k) { const w = CH[k]; if (w.clientWidth > 40 && w.clientHeight > 40) PLOT[k].setSize({ width: w.clientWidth, height: w.clientHeight }); }
Object.keys(CH).forEach(k => new ResizeObserver(() => fit(k)).observe(CH[k]));
function mountChart(bd, k) { bd.appendChild(CH[k]); CH[k].style.cssText = ''; requestAnimationFrame(() => fit(k)); }
function pushHistory(d) {
  const tt = (d.joints || []).map(j => j.temp);
  if (!tt.length) return;
  H.t.push(now()); H.soc.push(d.power ? d.power.soc : null);
  H.hot.push(Math.max(...tt)); H.avg.push(tt.reduce((a, b) => a + b) / tt.length);
  (d.feet || []).forEach((f, k) => H.feet[k] && H.feet[k].push(load(f)));
  while (H.t.length && H.t[0] < now() - 120) { H.t.shift(); H.soc.shift(); H.hot.shift(); H.avg.shift(); H.feet.forEach(a => a.shift()); }
  PLOT.batt.setData([H.t, H.soc]); PLOT.temp.setData([H.t, H.hot, H.avg]);
  PLOT.feet.setData([H.t, H.feet[1], H.feet[0], H.feet[3], H.feet[2]]);          // FL FR RL RR
}

/* ------------------------------------------------------------ live painters */
const recording = () => !!(S.rec && S.rec.walk && S.rec.walk.recording && S.rec.walk.recording.running);
function ring(frac, color) {
  const r = 20, c = 2 * Math.PI * r;
  return `<svg class="ring" viewBox="0 0 52 52"><circle cx="26" cy="26" r="${r}" fill="none" stroke="#26282b" stroke-width="5"/>
    <circle cx="26" cy="26" r="${r}" fill="none" stroke="${color}" stroke-width="5" stroke-dasharray="${c}" stroke-dashoffset="${c * (1 - frac)}" transform="rotate(-90 26 26)"/></svg>`;
}
function corners(d) {
  if (!d.feet || !d.joints) return;
  const total = d.feet.reduce((a, f) => a + load(f), 0) || 1;
  for (const [k, [j, f]] of Object.entries(LEGS)) {
    const el = E('c-' + k); if (!el) continue;
    const share = load(d.feet[f]) / total, lc = ramp(share * 2);      // 25% each is balanced
    const temps = [0, 1, 2].map(i => d.joints[j + i].temp);
    el.innerHTML = `<div class="nm"><b>${k}</b><span>${LEGNAME[k]}</span></div>
      <div class="body">${ring(Math.min(1, share * 2), lc)}<div class="load live-val"><b style="color:${lc}">${Math.round(share * 100)}</b><small>%</small><div>${Math.round(load(d.feet[f]))} N</div></div></div>
      <div class="jt">${['HIP', 'THIGH', 'KNEE'].map((n, i) => `<div style="background:${ramp((temps[i] - 20) / 60)}"><small>${n}</small>${Math.round(temps[i])}°</div>`).join('')}</div>`;
  }
}
function feetView(d) {
  const el = E('feetview'); if (!el || !d.feet) return;
  const total = d.feet.reduce((a, f) => a + load(f), 0) || 1;
  const pos = { FL: [52, 52], FR: [148, 52], RL: [52, 172], RR: [148, 172] };
  el.innerHTML = `<rect x="70" y="40" width="60" height="140" rx="10" fill="#18191b" stroke="#34373b" stroke-width="2"/>
    <path d="M86 44 h28 l-14 -14z" fill="#ff6a1a"/><text x="100" y="22" text-anchor="middle" fill="#7d8185" font-size="10" font-family="Bahnschrift" font-weight="600">FRONT</text>` +
    Object.entries(pos).map(([k, [x, y]]) => {
      const f = load(d.feet[LEGS[k][1]]), share = f / total, c = ramp(share * 2), r = 12 + 36 * share;
      return `<line x1="${x}" y1="${y}" x2="${x < 100 ? 72 : 128}" y2="${y < 100 ? 60 : 160}" stroke="#34373b" stroke-width="4"/>
        <circle cx="${x}" cy="${y}" r="${r}" fill="${c}" opacity=".2"/><circle cx="${x}" cy="${y}" r="${r * .62}" fill="${c}"/>
        <text x="${x}" y="${y + 5}" text-anchor="middle" fill="#0b0b0c" font-size="14" font-weight="700" font-family="Bahnschrift">${Math.round(share * 100)}%</text>
        <text x="${x}" y="${y + (y < 100 ? -r - 6 : r + 14)}" text-anchor="middle" fill="#b8bbbe" font-size="11" font-weight="600" font-family="Bahnschrift">${k}</text>`;
    }).join('');
}
function attitude(d) {
  const el = E('att'); if (!el || !d.imu) return;
  const roll = d.imu.roll, pitch = d.imu.pitch, py = pitch * 180 / Math.PI * 3;
  el.innerHTML = `<defs><clipPath id="ac"><circle cx="100" cy="100" r="86"/></clipPath></defs>
    <g clip-path="url(#ac)"><g transform="rotate(${-roll * 180 / Math.PI} 100 100) translate(0 ${py})">
      <rect x="-60" y="-80" width="320" height="180" fill="#1b2630"/><rect x="-60" y="100" width="320" height="180" fill="#27201a"/>
      <line x1="-60" y1="100" x2="260" y2="100" stroke="#f2f2ef" stroke-width="1.5"/>
      ${[-20, -10, 10, 20].map(p => `<line x1="${p % 20 ? 82 : 70}" y1="${100 - p * 3}" x2="${p % 20 ? 118 : 130}" y2="${100 - p * 3}" stroke="#7d8185" stroke-width="1.5"/><text x="136" y="${104 - p * 3}" fill="#7d8185" font-size="10" font-family="Bahnschrift">${Math.abs(p)}</text>`).join('')}
    </g></g><circle cx="100" cy="100" r="86" fill="none" stroke="#34373b" stroke-width="2"/>
    <path d="M58 100 h26 l8 8 l8 -8 h26" fill="none" stroke="#ff6a1a" stroke-width="3"/><circle cx="100" cy="100" r="3" fill="#ff6a1a"/>`;
  if (E('att-t')) E('att-t').textContent = `R ${(roll * 180 / Math.PI).toFixed(1)}° · P ${(pitch * 180 / Math.PI).toFixed(1)}°`;
}
function heatGrid(el, d, big) {
  if (!el || !d.joints) return;
  el.innerHTML = '<div></div><div class="h">HIP</div><div class="h">THIGH</div><div class="h">KNEE</div>' +
    ['FL', 'FR', 'RL', 'RR'].map(k => `<div class="leg">${k}</div>` + [0, 1, 2].map(i => { const tt = d.joints[LEGS[k][0] + i].temp; return `<div class="c" style="background:${ramp((tt - 20) / 60)}${big ? '' : '; padding:8px 4px; font-size:17px'}">${Math.round(tt)}°${big ? '<small>°C</small>' : ''}</div>`; }).join('')).join('');
}
function jointTable(el, d, units) {
  if (!el || !d.joints) return;
  el.innerHTML = '<tr><th>Joint</th><th style="text-align:right">Angle</th><th style="text-align:right">Torque</th><th style="text-align:right">Temp</th></tr>' +
    d.joints.map(j => `<tr><td>${esc(j.name)}</td><td class="n">${j.q.toFixed(2)}${units ? ' rad' : ''}</td><td class="n">${j.tau.toFixed(1)}${units ? ' N·m' : ''}</td><td class="n" style="color:${ramp((j.temp - 20) / 60)}">${Math.round(j.temp)}${units ? ' °C' : '°'}</td></tr>`).join('');
}

/* the trail: suggestion 4. Starts when the page opens, again at each walk. */
function trailPoint(d) {
  const p = d.motion && d.motion.position;
  if (!p) return;
  const last = trail[trail.length - 1];
  if (!last || Math.hypot(p[0] - last[0], p[1] - last[1]) > 0.04) { trail.push([p[0], p[1]]); if (trail.length > 6000) trail.splice(1, 1); }
}
function drawTrail() {
  const svg = E('trail'); if (!svg) return;
  let g = '';
  if (trail.length > 1) {
    const xs = trail.map(p => p[0]), ys = trail.map(p => p[1]);
    const minx = Math.min(...xs), maxx = Math.max(...xs), miny = Math.min(...ys), maxy = Math.max(...ys);
    const span = Math.max(maxx - minx, maxy - miny, 3), sc = 220 / span, cx = (minx + maxx) / 2, cy = (miny + maxy) / 2;
    const P = p => [150 + (p[0] - cx) * sc, 130 - (p[1] - cy) * sc];
    const step = [0.5, 1, 2, 5, 10, 20, 50].find(s => s * sc >= 28) || 50;
    for (let x = Math.floor((cx - 160 / sc) / step) * step; x <= cx + 160 / sc; x += step) { const X = 150 + (x - cx) * sc; g += `<line x1="${X}" y1="0" x2="${X}" y2="260" stroke="#1e2022"/>`; }
    for (let y = Math.floor((cy - 140 / sc) / step) * step; y <= cy + 140 / sc; y += step) { const Y = 130 - (y - cy) * sc; g += `<line x1="0" y1="${Y}" x2="300" y2="${Y}" stroke="#1e2022"/>`; }
    const s = P(trail[0]), e = P(trail[trail.length - 1]);
    g += `<polyline points="${trail.map(p => P(p).map(v => v.toFixed(1)).join(',')).join(' ')}" fill="none" stroke="#f2f2ef" stroke-width="2"/>`;
    g += `<path d="M${e[0]} ${e[1]} L${s[0]} ${s[1]}" stroke="#ff6a1a" stroke-dasharray="4 4"/>`;
    g += `<rect x="${s[0] - 5}" y="${s[1] - 5}" width="10" height="10" fill="none" stroke="#f2f2ef" stroke-width="2"/><circle cx="${e[0]}" cy="${e[1]}" r="5" fill="#ff6a1a"/>`;
    let walked = 0; for (let i = 1; i < trail.length; i++) walked += Math.hypot(trail[i][0] - trail[i - 1][0], trail[i][1] - trail[i - 1][1]);
    E('trail-d').textContent = Math.hypot(trail[0][0] - trail[trail.length - 1][0], trail[0][1] - trail[trail.length - 1][1]).toFixed(1) + ' m';
    E('trail-l').textContent = walked.toFixed(0) + ' m walked since ' + trailFrom;
    E('trail-g').textContent = 'grid ' + step + ' m';
  } else {
    g = `<text x="150" y="130" text-anchor="middle" fill="#7d8185" font-size="12" font-family="Bahnschrift">waiting for the robot to move</text>`;
    E('trail-d').textContent = '-'; E('trail-l').textContent = 'since ' + trailFrom; E('trail-g').textContent = '';
  }
  svg.innerHTML = g;
}

/* sensors card, with the LiDAR row that opens */
let lidarOpen = false;
function sensorRows() {
  const d = S.d || {}, sens = S.sens || {}, rows = [];
  const lid = sens.lidar || {}, lidC = pfCheck(/lidar/);
  rows.push({ lidar: true, ic: 'i-lidar', n: 'XT16 LiDAR', v: lid.pps, e: lid.pps_expected || 2500, u: 'pkt/s',
    ok: lid.on ? (lid.pps || 0) > (lid.pps_expected || 2500) * .6 : !!(lidC && lidC.ok), off: !lid.on && !(lidC && lidC.ok), note: lid.on ? '' : (lidC && lidC.ok ? 'answering; relay off' : 'no answer') });
  (d.streams || []).forEach(s => rows.push({ ic: /sport|motion/i.test(s.label + s.topic) ? 'i-odom' : 'i-imu', n: cap(s.label), v: s.hz, e: s.expected || s.hz || 1, u: 'Hz', ok: s.ok }));
  const cams = sens.cameras || {};
  rows.push({ ic: 'i-cam', n: 'Cameras', v: cams.on ? 1 : 0, e: 1, u: '', ok: cams.on, txt: cams.on ? 'on' : 'off', off: !cams.on });
  const dp = sens.depth || {};
  rows.push({ ic: 'i-cam', n: 'Depth camera', v: dp.on ? dp.fps : 0, e: dp.fps_expected || 15, u: 'fps', ok: dp.on && dp.fps > 0, off: !dp.on, txt: dp.on ? null : 'off' });
  return rows;
}
const cap = s => s ? s[0].toUpperCase() + s.slice(1) : s;
function sysList() {
  const el = E('sys-list'); if (!el) return;
  const lid = (S.sens && S.sens.lidar) || {}, lidarHost = (S.settings && S.settings.lidar && S.settings.lidar.host) || '192.168.123.20';
  el.innerHTML = sensorRows().map(r => {
    const val = r.txt != null ? r.txt : r.v != null ? (r.v >= 100 ? Math.round(r.v).toLocaleString() : (+r.v).toFixed(1)) : (r.note || '-');
    const frac = r.v != null && r.e ? Math.min(1, r.v / r.e) : (r.ok ? 1 : 0);
    return `<div class="sys ${r.lidar ? 'expand' : ''}" ${r.lidar ? 'data-lidar' : ''}><svg class="i ${r.off ? 't3' : ''}" style="width:18px;height:18px"><use href="#${r.ic}"/></svg>
      <div class="nm">${esc(r.n)}${r.lidar ? `<svg class="i chev ${lidarOpen ? 'open' : ''}" style="width:14px;height:14px"><use href="#i-chev"/></svg>` : ''}</div>
      <div class="v live-val ${r.off ? 't3' : r.ok ? '' : 'warn'}">${esc(val)}${r.v != null && r.txt == null && r.u ? `<small>${r.u}</small>` : ''}</div>
      <div class="bar"><i style="width:${frac * 100}%; background:${r.off ? '#2a2c2f' : r.ok ? 'var(--ok)' : 'var(--warn)'}"></i></div></div>` +
      (r.lidar && lidarOpen ? `<div class="more">
        <div class="stat-row" style="border-top:0"><span>Address</span><b>${esc(lidarHost)}</b></div>
        <div class="stat-row"><span>Copy to laptop</span><b>${lid.on ? 'on, ' + esc(lid.dest || '') : 'off'}</b></div>
        <div class="stat-row"><span>State</span><b style="font-size:12px">${esc(lid.note || '-')}</b></div>
        <a class="btn primary" style="width:100%; justify-content:center; margin-top:8px" href="http://${esc(lidarHost)}" target="_blank" rel="noopener"><svg class="i"><use href="#i-ext"/></svg>Manage settings</a>
        <div class="t3" style="font-size:11.5px; margin-top:5px">Opens the LiDAR's own page. Never change its destination IP.</div></div>` : '');
  }).join('');
}

/* camera */
let camNo = 2;
function camHost() {
  const v = (window.GO2_CAMERA_HOST || 'auto:8080').trim();
  return v.startsWith('auto') ? location.hostname + ':' + (v.split(':')[1] || '8080') : v;
}
function setCamera() {
  const img = E('camimg'); if (!img) return;
  $$('#camsel button').forEach(b => b.setAttribute('aria-pressed', +b.dataset.cam === camNo));
  img.onerror = () => { img.hidden = true; E('camnone').hidden = false; E('camnone').innerHTML = `No picture from ${esc(camHost())}.<br>The depth camera, when on, takes this camera over.`; };
  img.onload = () => { img.hidden = false; E('camnone').hidden = true; };
  img.hidden = false;
  img.src = `http://${camHost()}/stream?cam=${camNo}`;
}

/* quick record card and depth card */
function quickRec() {
  const el = E('qr'); if (!el) return;
  const pf = S.pf, w = S.rec && S.rec.walk && S.rec.walk.recording;
  if (recording()) {
    el.innerHTML = `<div class="recbadge"><i></i>RECORDING</div><div class="timer" style="font-size:50px; margin:6px 0 4px">${clock(w.seconds)}</div><div class="t3" style="font-weight:600; margin-bottom:12px">${esc(w.name)}</div>
      <button class="btn danger" style="width:100%; justify-content:center; padding:12px" data-a="stop"><svg class="i"><use href="#i-stop"/></svg>STOP</button>`;
    return;
  }
  const cs = ((pf && pf.checks) || []).filter(quietClock), ok = cs.filter(c => c.ok).length;
  el.innerHTML = `<div style="display:flex; align-items:center; gap:14px"><svg viewBox="0 0 110 110" style="width:88px; height:88px">${segs(cs, 55, 44, 7)}<text x="55" y="61" text-anchor="middle" fill="${pf && pf.ready ? '#3ccf6e' : '#f2f2ef'}" font-size="21" font-weight="700" font-family="Bahnschrift">${pf ? ok + '/' + cs.length : '-'}</text></svg>
     <div><div style="font-weight:600">${!pf ? 'Checking...' : pf.ready ? 'Ready to record' : 'Not ready'}</div><div class="t3" style="font-size:12.5px">${pf && !pf.ready ? esc(pf.blockers.join(', ')) : pf ? 'all checks pass' : ''}</div></div></div>
     <button class="btn primary" style="width:100%; justify-content:center; padding:12px; margin-top:12px; font:700 15px var(--num); letter-spacing:.1em" data-a="start" ${pf && pf.ready ? '' : 'disabled'}><svg class="i"><use href="#i-play"/></svg>START WALK</button>`;
}
function segs(cs, c, r, w) {
  if (!cs.length) return `<circle cx="${c}" cy="${c}" r="${r}" fill="none" stroke="#26282b" stroke-width="${w}"/>`;
  const seg = 2 * Math.PI / cs.length;
  return cs.map((x, i) => `<path d="${arcPath(c, c, r, -Math.PI / 2 + i * seg + .05, -Math.PI / 2 + (i + 1) * seg - .05)}" stroke="${x.ok ? '#3ccf6e' : x.blocking ? '#ff453a' : '#f2c200'}" stroke-width="${w}" fill="none"/>`).join('');
}
function depthCard() {
  const dp = (S.sens && S.sens.depth) || {};
  gauge(E('g-depth'), { value: dp.on ? (dp.fps || 0) : 0, min: 0, max: Math.max(20, (dp.fps_expected || 15) * 1.3), unit: '', sub: dp.on ? 'fps · ' + (dp.note || '') : 'off', color: dp.on ? '#3ccf6e' : '#3a3d41' });
  const b = E('depth-btn');
  if (b) b.innerHTML = sensBusy() ? `<span class="t3" style="font-size:12px">${esc(S.sens.job.title)}...</span>` :
    dp.on ? '<button class="btn sm" data-a="depth-off">Turn off</button>' : '<button class="btn primary sm" data-a="depth-on">Turn on</button>';
}
const sensBusy = () => !!(S.sens && S.sens.job && S.sens.job.running);

/* walks tables */
function walksRows(target, compact) {
  const el = E(target); if (!el) return;
  const ws = (S.rec && S.rec.walk && S.rec.walk.walks) || [];
  if (!ws.length) { el.innerHTML = '<tr><td class="t3" style="padding:18px 10px">No walks on the robot yet.</td></tr>'; return; }
  const cur = recording() ? S.rec.walk.recording.name : null;
  el.innerHTML = `<tr><th>Name</th>${compact ? '' : '<th>Recorded</th>'}<th style="text-align:right">Length</th><th style="text-align:right">Size</th><th></th></tr>` +
    ws.map(w => `<tr><td>${esc(w.name)}${w.complete && !w.clock_ok ? ' <span class="chip warn" title="Recorded before the clock was set; it cannot be lined up">clock not set</span>' : ''}</td>${compact ? '' : `<td class="t2">${w.modified ? new Date(w.modified * 1000).toLocaleString(undefined, { day: 'numeric', month: 'short', hour: '2-digit', minute: '2-digit' }) : '-'}</td>`}
      <td class="n">${clock(w.seconds)}</td><td class="n">${fmtB(w.bytes)}</td>
      <td class="act">${w.name === cur ? '<span class="bad" style="font:700 12px var(--num)">RECORDING</span>' : `<a class="btn sm" href="/api/walk-download?name=${encodeURIComponent(w.name)}" download><svg class="i"><use href="#i-down"/></svg>Download</a> <button class="btn ghost sm" data-del="${esc(w.name)}" title="Delete from the robot"><svg class="i"><use href="#i-trash"/></svg></button>`}</td></tr>`).join('');
  if (E('walks-sum')) E('walks-sum').textContent = ws.length + ' walks · ' + fmtB(ws.reduce((a, w) => a + (w.bytes || 0), 0));
}

/* ------------------------------------------------------------ header and alerts */
// A clock that is right is not news: it is shown only when it is wrong.
const quietClock = c => !(/clock/.test(c.headline) && c.ok);
function pfCheck(re) { return S.pf && S.pf.checks ? S.pf.checks.find(c => re.test(c.headline)) : null; }
function paintHeader() {
  const d = S.d;
  E('mode').textContent = d && d.motion ? (d.motion.mode_name || '').toUpperCase() : S.online ? 'NO MOTION DATA' : '-';
  const lp = E('linkpill');
  if (!S.online) lp.innerHTML = '<i class="led bad"></i><b>LINK</b> lost';
  else if (d && d.connected) lp.innerHTML = `<i class="led"></i><b>LINK</b> ${Math.round((d.age || 0) * 1000)} ms`;
  else lp.innerHTML = '<i class="led warn"></i><b>LINK</b> no robot data';
  const cc = pfCheck(/clock/), cp = E('clockpill');
  cp.style.display = cc && !cc.ok ? '' : 'none';
  cp.className = 'pill' + (cc && !cc.ok ? ' warn' : '');
  cp.innerHTML = !cc ? '<i class="led off"></i><b>CLOCK</b> -' : cc.ok ? `<i class="led"></i><b>CLOCK</b> ${new Date().toTimeString().slice(0, 5)}`
    : '<svg class="i" style="width:15px;height:15px"><use href="#i-clock"/></svg><b>CLOCK</b> not set';
  const w = S.rec && S.rec.walk && S.rec.walk.recording;
  E('recpill').innerHTML = recording() ? `<i class="led bad rec"></i><b>REC</b> <span class="bad">${clock(w.seconds)}</span>` : '<i class="led off"></i><b>REC</b> idle';
  E('rec-badge').style.display = S.pf && !S.pf.ready && !recording() ? '' : 'none';
  E('alert').classList.toggle('hide', !(cc && !cc.ok));
  if (cc && !cc.ok) E('alert-t').innerHTML = `<b>Robot clock reads ${esc((cc.detail.match(/reads ([\d-]+)/) || [, '1970'])[1])}.</b> <span class="t2">${clockTried ? 'Setting it from this device did not work: ' + esc(clockErr || 'see the log') : 'Setting it from this device...'}</span>`;
  document.body.classList.toggle('offline', !S.online);
  if (!S.online) E('offline-t').textContent = (wasRecording ? 'The walk keeps recording on the robot. ' : '') + 'Last answer ' + Math.round((Date.now() - S.lastOk) / 1000) + ' s ago.';
}

/* suggestion 1: set the robot's clock without being asked, once per page */
let clockTried = false, clockErr = null;
async function autoClock() {
  const cc = pfCheck(/clock/);
  if (!cc || cc.ok || clockTried || recording()) return;
  if (new Date().getFullYear() < 2026) { clockTried = true; clockErr = "this device's own clock looks wrong"; return; }
  clockTried = true;
  try {
    const r = await control('set-clock', { epoch_ms: Date.now() });
    if (r.preflight) S.pf = r.preflight;
  } catch (e) { clockErr = e.message; }
  paintHeader();
}

/* ------------------------------------------------------------ record page */
const OPT = { depth: false, color: false, relay: false, res: '640x480', fps: 15, stop: 0 };
let optsInit = false;
function modes() { return (S.sens && S.sens.modes) || [{ id: '640x480x15', proven: true }]; }
function paintOptions() {
  const sens = S.sens || {}, dp = sens.depth || {}, lid = sens.lidar || {};
  if (!optsInit && S.sens) {
    optsInit = true;
    const want = sens.wanted || {};
    const m = (want.depth_mode || '640x480x15').split('x');
    OPT.res = m[0] + 'x' + m[1]; OPT.fps = +m[2]; OPT.color = !!want.color;
  }
  OPT.depth = !!dp.on; OPT.relay = !!lid.on;
  const res = [...new Set(modes().map(m => m.id.split('x').slice(0, 2).join('x')))];
  const sel = E('o-res');
  if (sel.dataset.v !== res.join()) { sel.innerHTML = res.map(r => `<option value="${r}">${r.replace('x', ' × ')}</option>`).join(''); sel.dataset.v = res.join(); }
  sel.value = OPT.res;
  const fpsList = modes().filter(m => m.id.startsWith(OPT.res + 'x')).map(m => ({ fps: +m.id.split('x')[2], proven: m.proven }));
  E('o-fps').innerHTML = fpsList.map(f => `<button data-fps="${f.fps}" aria-pressed="${f.fps === OPT.fps}" title="${f.proven ? '' : 'not yet proven on this camera'}">${f.fps} fps${f.proven ? '' : ' *'}</button>`).join('');
  const locked = sensBusy();
  const chosen = OPT.res + 'x' + OPT.fps, pending = dp.on && !locked && (chosen !== dp.mode || OPT.color !== !!dp.color);
  E('depth-apply').hidden = !pending;
  if (pending) E('depth-apply-t').textContent = `running at ${(dp.mode || '').replace(/x(\d+)$/, ', $1 fps').replace('x', ' × ')}${dp.color ? ' with colour' : ''}; Apply restarts the camera (about 20 s)`;
  sel.disabled = locked;
  $$('#o-fps button').forEach(b => b.disabled = locked);
  [['o-depth', OPT.depth], ['o-color', OPT.color], ['o-relay', OPT.relay]].forEach(([id, on]) => { const t = E(id); t.setAttribute('aria-checked', on); t.querySelector('.tl').textContent = on ? 'On' : 'Off'; });
  E('o-color').style.opacity = locked ? .5 : 1;
  $$('[data-needs="depth"]').forEach(r => r.style.opacity = OPT.depth ? 1 : .55);
  E('depth-note').textContent = dp.on ? (dp.fps ? Math.round(dp.fps) + ' fps now. ' : '') + (dp.note || '') + '' : 'RealSense depth, lossless. Off.';
  E('sens-job').textContent = sensBusy() ? sens.job.title + '... ' + (sens.job.percent != null ? sens.job.percent + '%' : '') : '';
  $$('#o-stop button').forEach(b => b.setAttribute('aria-pressed', +b.dataset.v === OPT.stop));
  const [w, h] = OPT.res.split('x').map(Number), k = (w * h) / (640 * 480) * OPT.fps / 15;
  const perMin = 160 + (OPT.depth ? 180 * k + (OPT.color ? 60 * k : 0) : 0);
  E('est-rate').textContent = perMin >= 1000 ? (perMin / 1000).toFixed(1) + ' GB' : Math.round(perMin) + ' MB';
  const free = S.sys && S.sys.disk_free;
  if (free) { const mins = free / 1e6 / perMin; E('est-fit').textContent = mins >= 60 ? Math.floor(mins / 60) + ' h' : Math.round(mins) + ' min'; }
}
function paintRecord() {
  const pf = S.pf, cs = ((pf && pf.checks) || []).filter(quietClock), ok = cs.filter(c => c.ok).length;
  E('pf-ring').innerHTML = segs(cs, 105, 88, 12);
  E('pf-n').textContent = pf ? ok + '/' + cs.length : '-';
  E('pf-n').className = pf && pf.ready ? 'ok' : '';
  E('pf-l').textContent = !pf ? 'CHECKING' : pf.ready ? 'READY' : 'NOT READY';
  E('checks').innerHTML = cs.map(c => `<div class="chk ${c.ok ? '' : 'bad'}"><div class="ic"><svg class="i"><use href="#i-${c.ok ? 'check' : 'x'}"/></svg></div><div class="nm">${esc(cap(c.headline))}</div><div class="v">${c.ok ? esc(String(c.detail).slice(0, 26)) : ''}</div>
    ${c.ok ? '' : `<div class="fx">${esc(String(c.detail).split('. ')[0])}.${/clock/.test(c.headline) ? ' <a href="#" data-a="setclock" style="color:#ffd48a">Set from this device</a>' : ''}</div>`}</div>`).join('') || '<div class="empty"><b>Waiting for the robot</b></div>';
  const rec = recording(), w = rec && S.rec.walk.recording;
  E('launch').hidden = rec; E('live-rec').hidden = !rec;
  if (!rec) {
    if (!E('w-name').value) { const d = new Date(); E('w-name').value = `${d.getFullYear()}-${String(d.getMonth() + 1).padStart(2, '0')}-${String(d.getDate()).padStart(2, '0')}_walk-1`; }
    E('start').disabled = !pf || !S.online;
    E('startwhy').innerHTML = !pf ? 'Waiting for the robot.' : pf.ready ? 'All checks pass. Stand clear.' : 'First: ' + esc(pf.blockers.join(', ')) + '. <a href="#" data-a="force" style="color:var(--text2)">Start anyway</a>';
    E('start').disabled = !pf || !pf.ready;
    paintOptions();
  } else {
    E('timer').textContent = clock(w.seconds); E('rec-name').textContent = w.name;
    E('minis').innerHTML = (w.health || []).map(h => `<div class="mini"><svg viewBox="0 0 110 80"><path d="${arcPath(55, 58, 42, Math.PI, 2 * Math.PI)}" stroke="#26282b" stroke-width="7" fill="none"/><path d="${arcPath(55, 58, 42, Math.PI, Math.PI + Math.PI * Math.min(1, h.rate / (h.expected || 1)))}" stroke="${h.ok ? '#3ccf6e' : '#ff453a'}" stroke-width="7" fill="none"/></svg><div class="v">${Math.round(h.rate).toLocaleString()}</div><div class="l">${esc(cap(h.name))}</div><div class="s">${esc(h.unit)} · expected ${Math.round(h.expected).toLocaleString()}</div></div>`).join('');
    E('rec-size').textContent = fmtB(w.bytes_on_disk) + ' written';
    E('rec-left').textContent = w.minutes_left != null ? 'room for ' + (w.minutes_left >= 60 ? Math.floor(w.minutes_left / 60) + ' h more' : Math.round(w.minutes_left) + ' min more') : '';
  }
  walksRows('walks-t');
}
function showReport(v, warning) {
  if (!v) return;
  const vv = v.verdict || {};
  E('report').hidden = false;
  E('report-t').innerHTML = `${esc(v.name)} ${vv.ok ? '<span class="ok">is good to map</span>' : '<span class="warn">has problems</span>'}`;
  E('report-b').innerHTML = (warning ? `<div class="t2" style="margin-bottom:8px">${esc(warning)}</div>` : '') +
    `<div class="t2" style="font-size:13px; margin-bottom:6px">${clock(v.seconds)} · ${fmtB(v.bytes)} · ${vv.passed}/${vv.total} checks</div>` +
    (vv.checks || []).filter(quietClock).map(c => `<div class="chk ${c.ok ? '' : 'bad'}"><div class="ic"><svg class="i"><use href="#i-${c.ok ? 'check' : 'x'}"/></svg></div><div class="nm">${esc(cap(c.headline))}</div><div class="v">${esc(c.detail)}</div></div>`).join('') +
    `<div class="t3" style="font-size:12.5px; margin-top:10px">Download it below, or with Go2 Studio on a laptop.</div>`;
}
async function startWalk(force) {
  const name = E('w-name').value.trim();
  if (!/^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$/.test(name)) { toast('Name: letters, digits, dash and underscore; no spaces.', true); return; }
  E('start').disabled = true;
  E('startwhy').textContent = 'Starting the recorder...';
  try {
    S.rec = await control('walk-start', { name, duration: OPT.stop * 60, force: !!force });
    E('report').hidden = true;
    trail = []; trailFrom = 'the walk started';
    toast('Recording ' + name + '. Unplug the cable once the counters move.');
  } catch (e) {
    if (e.status === 409 && e.body && e.body.preflight) ask('Start anyway?', 'Not ready: ' + e.body.preflight.blockers.join(', ') + '. The walk will probably not be usable.', 'Start anyway', () => startWalk(true));
    else toast(e.message, true);
  }
  paintAll();
}
async function stopWalk() {
  try {
    const r = await control('walk-stop');
    S.rec = r;
    showReport(r.verdict, r.warning);
    toast('Stopped.');
  } catch (e) { toast(e.message, true); }
  paintAll();
}

/* ------------------------------------------------------------ sensors, details, system */
function sensorsPage() {
  const sens = S.sens || {}, dp = sens.depth || {}, lid = sens.lidar || {};
  const lidarHost = (S.settings && S.settings.lidar && S.settings.lidar.host) || '192.168.123.20';
  const rows = sensorRows();
  E('sensor-cards').innerHTML = rows.map((r, i) => `<div class="card"><div class="hd"><svg class="i ${r.off ? 't3' : ''}"><use href="#${r.ic}"/></svg><span class="cap">${esc(r.n)}</span><span class="r"><i class="led ${r.off ? 'off' : r.ok ? '' : 'warn'}"></i></span></div>
    <div class="bd"><div class="gauge" data-si="${i}"></div>
    ${r.lidar ? `<div class="t3" style="text-align:center; font-size:12.5px">${esc(lidarHost)} · ${esc(lid.note || '')}</div>
      <div style="display:flex; gap:8px; justify-content:center; margin-top:10px"><a class="btn primary sm" href="http://${esc(lidarHost)}" target="_blank" rel="noopener"><svg class="i"><use href="#i-ext"/></svg>Manage settings</a>
      <button class="btn sm" data-a="${lid.on ? 'relay-off' : 'relay-on'}">${lid.on ? 'Stop copy to laptop' : 'Copy to laptop'}</button></div>
      <div class="t3" style="text-align:center; font-size:11.5px; margin-top:6px">The LiDAR's own page. Never change its destination IP.</div>` : ''}
    ${r.n === 'Depth camera' ? `<div class="t3" style="text-align:center; font-size:12.5px">${esc(dp.note || '')}</div><div style="text-align:center; margin-top:10px">${sensBusy() ? esc(sens.job.title) + '...' : dp.on ? '<button class="btn sm" data-a="depth-off">Turn off</button>' : '<button class="btn primary sm" data-a="depth-on">Turn on</button>'}</div>` : ''}
    </div></div>`).join('');
  rows.forEach((r, i) => gauge($(`[data-si="${i}"]`), { value: r.txt != null && r.v == null ? (r.ok ? 1 : 0) : (r.v || 0), min: 0, max: (r.e || 1) * 1.3, unit: '', sub: r.off ? 'off' : (r.u ? r.u + ' · expected ' + Math.round(r.e).toLocaleString() : (r.ok ? 'on' : 'off')), color: r.off ? '#3a3d41' : r.ok ? '#3ccf6e' : '#f2c200', dec: r.v != null && r.v < 100 && r.v % 1 ? 1 : 0 }));
}
async function loadSystem() {
  try { S.sys = await get('/api/system', 6000); } catch (e) { return; }
  const y = S.sys, gb = b => b == null ? '-' : (b / 1e9).toFixed(1) + ' GB';
  const up = y.uptime != null ? (y.uptime >= 3600 ? Math.floor(y.uptime / 3600) + ' h ' : '') + Math.floor(y.uptime % 3600 / 60) + ' min' : '-';
  E('autostart').textContent = y.autostart ? 'autostart ' + y.autostart : 'autostart not installed';
  const P = [['dashboard', 'Dashboard and recorder', 'i-live'], ['lidar_relay', 'LiDAR relay', 'i-lidar'], ['camera_server', 'Camera server', 'i-cam'], ['realsense_camera', 'Depth camera', 'i-cam'], ['recorder', 'Walk recorder', 'i-rec']];
  E('programs').innerHTML = P.map(([k, n, ic]) => { const on = y.programs[k]; return `<div class="svc"><div class="ic" style="${on ? '' : 'color:var(--text3)'}"><svg class="i"><use href="#${ic}"/></svg></div><div><div class="nm">${n}</div><div class="sub"><span class="${on ? 'ok' : 't3'}">${on ? 'running' : on === false ? 'not running' : 'unknown'}</span></div></div><div>${k !== 'recorder' ? `<button class="btn sm" data-log="${k}">Log</button>` : ''}</div></div>`; }).join('');
  const tile = (l, v, s) => `<div class="mini" style="padding:12px"><div class="t3" style="font:600 11.5px var(--num)">${l}</div><div class="num" style="font-size:26px">${v}<small class="t2" style="font-size:14px">${s || ''}</small></div></div>`;
  E('sysinfo').innerHTML = `<div class="grid2" style="gap:10px">
      ${tile('CPU', y.load && y.cpus ? Math.round(y.load[0] / y.cpus * 100) : '-', '%')}${tile('MEMORY', y.mem_total ? ((y.mem_total - y.mem_available) / 1e9).toFixed(1) : '-', ' / ' + gb(y.mem_total))}
      ${tile('TEMPERATURE', y.temp_c ?? '-', ' °C')}${tile('DISK FREE', y.disk_free ? Math.round(y.disk_free / 1e9) : '-', ' GB')}</div>
    <div class="stat-row" style="margin-top:10px"><span>Dashboard version</span><b style="font-size:13.5px">${esc(y.version || '-')}</b></div>
    <div class="stat-row"><span>Computer</span><b style="font-size:13.5px">${esc(y.model || '-')}</b></div>
    <div class="stat-row"><span>Up for</span><b style="font-size:13.5px">${up}</b></div>
    <div class="stat-row"><span>Addresses</span><b style="font:12.5px var(--mono)">${esc((y.addresses || []).join(', '))}</b></div>`;
}
let logName = 'dashboard';
async function loadLog() {
  E('log-seg').innerHTML = ['dashboard', 'lidar_relay', 'camera_server', 'realsense_camera'].map(n => `<button data-log="${n}" aria-pressed="${n === logName}">${n}</button>`).join('');
  E('log-t').textContent = 'Log · ' + logName;
  try {
    const r = await get('/api/logs?name=' + logName);
    E('log').textContent = r.error ? r.error : (r.lines || []).join('\n') || '(empty)';
    E('log').scrollTop = E('log').scrollHeight;
  } catch (e) { E('log').textContent = 'Could not read the log: ' + e.message; }
}
async function loadCardLog() {
  const el = E('cardlog'); if (!el) return;
  try { const r = await get('/api/logs?name=dashboard'); el.textContent = (r.lines || []).slice(-60).join('\n'); el.scrollTop = el.scrollHeight; } catch (e) { /* next time */ }
}

/* ------------------------------------------------------------ paint everything */
function paintAll() {
  paintHeader();
  const d = S.d;
  if (d) {
    corners(d); feetView(d); attitude(d); heatGrid(E('heat'), d); jointTable(E('jtab'), d);
    if (!E('p-details').hidden) { heatGrid(E('dheat'), d, true); jointTable(E('djtab'), d, true); }
    const js = d.joints || [];
    const hot = js.length ? js.reduce((a, j) => j.temp > a.temp ? j : a) : null;
    gauge(E('g-batt'), { value: d.power ? d.power.soc : null, min: 0, max: 100, unit: '%', sub: 'charge', bands: BATT_BANDS });
    gauge(E('g-temp'), { value: hot ? hot.temp : null, min: 20, max: 90, unit: '°C', sub: hot ? hot.name : 'no data', color: hot && hot.temp > 70 ? '#ff453a' : hot && hot.temp > 60 ? '#f2c200' : null, hi: 70 });
    const v = d.motion && d.motion.velocity ? Math.hypot(d.motion.velocity[0], d.motion.velocity[1]) : null;
    gauge(E('g-speed'), { value: v, min: 0, max: 2, unit: ' m/s', sub: 'ground speed', dec: 2 });
    if (E('volts')) { E('volts').textContent = d.power ? d.power.volts.toFixed(1) + ' V' : '-'; E('amps').textContent = d.power ? d.power.amps.toFixed(1) + ' A' : '-'; E('tleft').textContent = timeLeft(); }
    if (E('height')) { E('height').textContent = d.motion ? d.motion.body_height.toFixed(2) + ' m' : '-'; E('gait').textContent = d.motion ? cap(d.motion.gait || '-') : '-'; }
    if (E('now-batt') && d.power) E('now-batt').innerHTML = d.power.soc + '<small>%</small>';
  }
  sysList(); drawTrail(); quickRec(); depthCard(); walksRows('cardwalks', true);
  if (!E('p-record').hidden) paintRecord();
  if (!E('p-sensors').hidden) sensorsPage();
}
function timeLeft() {
  if (SOC.length < 2) return 'measuring...';
  const [t0, s0] = SOC[0], [t1, s1] = SOC[SOC.length - 1];
  if (t1 - t0 < 90 || s0 - s1 < 1) return 'measuring...';
  const perMin = (s0 - s1) / ((t1 - t0) / 60);
  const mins = s1 / perMin;
  return '~ ' + (mins >= 60 ? Math.floor(mins / 60) + ' h ' + Math.round(mins % 60) + ' min' : Math.round(mins) + ' min');
}

/* ------------------------------------------------------------ polling */
let lastHist = 0, frameN = 0;
async function pollData() {
  try {
    const d = await get('/data', 2500);
    S.d = d; S.online = true; S.fails = 0; S.lastOk = Date.now();
    robot.update(d);
    trailPoint(d);
    if (d.power) { SOC.push([Date.now() / 1000, d.power.soc]); while (SOC.length && SOC[0][0] < Date.now() / 1000 - 600) SOC.shift(); }
    if (Date.now() - lastHist > 500) { lastHist = Date.now(); pushHistory(d); }
    if (frameN++ % 2 === 0) paintAll();
  } catch (e) {
    if (++S.fails >= 3) { S.online = false; paintHeader(); }
  }
  setTimeout(pollData, S.online ? 200 : 2000);
}
async function pollSlow() {
  try {
    const [pf, rec, sens] = await Promise.all([get('/api/preflight'), get('/api/recordings'), get('/api/sensors')]);
    S.pf = pf; S.rec = rec; S.sens = sens;
    const r = recording();
    if (r && !wasRecording) { trail = []; trailFrom = 'the walk started'; }
    if (!r && wasRecording && rec.walk && rec.walk.last_verdict) showReport(rec.walk.last_verdict);
    wasRecording = r;
    autoClock();
    if (!document.body.classList.contains('edit')) paintAll();
  } catch (e) { /* the fast poll reports being offline */ }
}

/* ------------------------------------------------------------ clicks */
document.addEventListener('click', async e => {
  const t = e.target.closest('[data-a],[data-rm],[data-add],[data-preset],[data-move],[data-act],[data-goto],[data-lidar],[data-cam],[data-log],[data-del],[data-fps],[data-m-close]');
  if (!t) return;
  if (t.hasAttribute('data-m-close')) { closeModal(); return; }
  if (t.dataset.rm) { const el = $(`.grid-stack-item[gs-id="${t.dataset.rm}"]`); if (el) { park(t.dataset.rm); grid.removeWidget(el); saveLayout(); catalog(); } return; }
  if (t.dataset.add) { const c = CARDS[t.dataset.add]; addCard(t.dataset.add, undefined, undefined, c.w, c.h); grid.compact(); layoutName = 'Custom'; saveLayout(); catalog(); toast(c.title + ' added. Drag it where you want it.'); return; }
  if (t.dataset.preset) { applyLayout(PRESETS[t.dataset.preset].items, t.dataset.preset); saveLayout(); closeDrawer(); toast('Layout: ' + t.dataset.preset); return; }
  if (t.dataset.move) { askMove(t.dataset.move); return; }
  if (t.dataset.act === 'stop') { sport('StopMove'); return; }
  if (t.dataset.goto) { $(`.rail button[data-p="${t.dataset.goto}"]`).click(); return; }
  if (t.hasAttribute('data-lidar')) { lidarOpen = !lidarOpen; sysList(); return; }
  if (t.dataset.cam) { camNo = +t.dataset.cam; setCamera(); return; }
  if (t.dataset.log) { logName = t.dataset.log; if (E('p-system').hidden) $('.rail button[data-p="system"]').click(); else loadLog(); return; }
  if (t.dataset.del) { const n = t.dataset.del; ask('Delete ' + n + '?', 'It is deleted from the robot. If it is not on a laptop yet, it is gone for good.', 'Delete', async () => { try { S.rec = await control('walk-delete', { name: n }); toast('Deleted ' + n + '.'); paintAll(); } catch (err) { toast(err.message, true); } }); return; }
  if (t.dataset.fps) { OPT.fps = +t.dataset.fps; paintOptions(); return; }
  const a = t.dataset.a;
  if (!a) return;
  e.preventDefault();
  if (a === 'edit') { setEdit(true); }
  if (a === 'done') { setEdit(false); closeDrawer(); saveLayout(); toast('Layout saved on the robot.'); }
  if (a === 'add') { catalog(); E('scrim').hidden = false; E('drawer').hidden = false; }
  if (a === 'close') closeDrawer();
  if (a === 'reset') { applyLayout(PRESETS.Mission.items, 'Mission'); saveLayout(); toast('Back to the Mission layout.'); }
  if (a === 'setclock') { try { const r = await control('set-clock', { epoch_ms: Date.now() }); if (r.preflight) S.pf = r.preflight; toast('Robot clock set from this device.'); } catch (err) { clockErr = err.message; toast(err.message, true); } paintAll(); }
  if (a === 'start') startWalk(false);
  if (a === 'force') startWalk(true);
  if (a === 'stop') stopWalk();
  if (a === 'report-close') E('report').hidden = true;
  if (a === 'trail-reset') { trail = []; trailFrom = new Date().toTimeString().slice(0, 5); drawTrail(); }
  if (a === 'log-refresh') loadLog();
  if (a === 'depth-undo') { const m = ((S.sens && S.sens.depth && S.sens.depth.mode) || '640x480x15').split('x'); OPT.res = m[0] + 'x' + m[1]; OPT.fps = +m[2]; OPT.color = !!(S.sens.depth && S.sens.depth.color); paintOptions(); }
  if (a === 'depth-apply') {
    if (recording()) { toast('A walk is recording. Stop it first: restarting the camera would leave a gap in its depth.', true); return; }
    try { S.sens = await control('sensors-depth-on', { mode: OPT.res + 'x' + OPT.fps, color: OPT.color }); toast('Restarting the depth camera at the new setting (about 20 s).'); } catch (err) { toast(err.message, true); }
    paintAll();
  }
  if (a === 'depth-on' || a === 'depth-off' || a === 'relay-on' || a === 'relay-off') {
    const act = { 'depth-on': 'sensors-depth-on', 'depth-off': 'sensors-depth-off', 'relay-on': 'sensors-lidar-on', 'relay-off': 'sensors-lidar-off' }[a];
    try { S.sens = await control(act, a === 'depth-on' ? { mode: OPT.res + 'x' + OPT.fps, color: OPT.color } : {}); toast(a === 'depth-on' ? 'Starting the depth camera (about 20 s).' : 'Done.'); } catch (err) { toast(err.message, true); }
    paintAll();
  }
});
E('start').onclick = () => startWalk(false);
E('o-depth').onclick = async () => {
  if (sensBusy()) { toast('The camera is still starting or stopping.', true); return; }
  const on = !OPT.depth;
  try {
    S.sens = await control(on ? 'sensors-depth-on' : 'sensors-depth-off', on ? { mode: OPT.res + 'x' + OPT.fps, color: OPT.color } : {});
    toast(on ? 'Starting the depth camera (about 20 s).' : 'Stopping the depth camera.');
  } catch (err) { toast(err.message, true); }
  paintAll();
};
E('o-color').onclick = () => { if (sensBusy()) { toast('The camera is still starting or stopping.', true); return; } OPT.color = !OPT.color; paintOptions(); };
E('o-relay').onclick = () => control(OPT.relay ? 'sensors-lidar-off' : 'sensors-lidar-on').then(r => { S.sens = r; paintAll(); }).catch(err => toast(err.message, true));
E('o-res').onchange = e => { OPT.res = e.target.value; const f = modes().filter(m => m.id.startsWith(OPT.res + 'x')).map(m => +m.id.split('x')[2]); if (!f.includes(OPT.fps)) OPT.fps = f[0]; paintOptions(); };
$$('#o-stop button').forEach(b => b.onclick = () => { OPT.stop = +b.dataset.v; paintOptions(); });

/* ------------------------------------------------------------ start */
(async () => {
  let layout = null;
  try { const r = await get('/api/settings'); S.settings = r.settings; layout = r.settings && r.settings.v2_layout; } catch (e) { /* use this device's copy */ }
  if (!layout) { try { layout = JSON.parse(localStorage.getItem('go2.v2.layout') || 'null'); } catch (e) { /* none */ } }
  applyLayout(layout && layout.items && layout.items.length ? layout.items.filter(i => CARDS[i[0]]) : PRESETS.Mission.items, layout ? layout.name : 'Mission');
  pollData(); pollSlow();
  setInterval(pollSlow, 2000);
  setInterval(loadSystem, 10000); loadSystem();
  setInterval(() => { if (E('cardlog')) loadCardLog(); if (!E('p-system').hidden) loadLog(); }, 5000);
})();
