'use strict';
/* Go2 Studio page.
 *
 * Everything here reads from the local Studio server (core/server.py):
 *   GET  /api/state   once a second: robot, checks, jobs, settings, disk
 *   GET  /api/walks   every few seconds: walks here and on the robot
 *   POST /api/robot, /api/walks, /api/os, /api/settings: the buttons
 * The server does everything a page is not allowed to (Explorer, Foxglove,
 * the folder dialog, running the conversion); this file only draws.
 */

const $ = (s, r = document) => r.querySelector(s);
const $$ = (s, r = document) => [...r.querySelectorAll(s)];
const esc = v => String(v ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const icon = (id, cls = '') => `<svg class="i ${cls}"><use href="#i-${id}"/></svg>`;

function fmtB(b) {
  if (b == null) return '?';
  if (b >= 1e9) return (b / 1e9).toFixed(b >= 1e10 ? 0 : 1) + ' GB';
  if (b >= 1e6) return Math.round(b / 1e6) + ' MB';
  if (b >= 1e3) return Math.round(b / 1e3) + ' kB';
  return b + ' B';
}
function fmtDur(s) {
  s = Math.round(s || 0);
  if (s < 60) return s + ' s';
  const m = Math.floor(s / 60), r = s % 60;
  return m + ' min' + (r ? ' ' + String(r).padStart(2, '0') + ' s' : '');
}
function fmtEta(s) { return s == null ? '' : s < 60 ? 'under 1 min' : 'about ' + Math.round(s / 60) + ' min left'; }
function fmtDate(t) {
  if (!t || t < 1.6e9) return 'date unknown';
  return new Date(t * 1000).toLocaleString(undefined, { day: 'numeric', month: 'short', year: 'numeric', hour: '2-digit', minute: '2-digit' });
}
function clock(s) { s = Math.max(0, Math.floor(s || 0)); return String(Math.floor(s / 60)).padStart(2, '0') + ':' + String(s % 60).padStart(2, '0'); }

/* ------------------------------------------------------------------ API */
const api = {
  async get(path) {
    const r = await fetch(path, { cache: 'no-store' });
    if (!r.ok) throw new Error(path + ' ' + r.status);
    return r.json();
  },
  async post(path, body) {
    const r = await fetch(path, { method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Go2-Studio': '1' }, body: JSON.stringify(body || {}) });
    let j = null;
    try { j = await r.json(); } catch (e) { /* not JSON */ }
    if (!r.ok) throw new Error((j && j.error) || (path + ' ' + r.status));
    return j;
  },
};
async function act(path, body, okText) {
  try {
    const r = await api.post(path, body);
    if (okText) toast(okText);
    return r;
  } catch (e) {
    toast(e.message, { bad: true });
    return null;
  }
}
window.addEventListener('error', e => { api.post('/api/log', { level: 'err', msg: 'page: ' + e.message + ' at ' + (e.filename || '') + ':' + e.lineno }).catch(() => {}); });

/* ---------------------------------------------------------------- state */
let S = null;                          // /api/state
let W = { walks: [], robot_reachable: false };
let page = localStorage.getItem('go2.page') || 'record';
if (page === 'robot') page = 'live';            // the Robot page became Live (the robot's own dashboard)
const R = key => S && S.robot.data[key] ? S.robot.data[key].body : null;
const Rage = key => S && S.robot.data[key] ? S.robot.data[key].age : null;
const dev = () => !!(S && S.settings.dev);
const robotName = () => (S ? S.robot.base.replace('http://', '') : '');

let engineJobWas = null;
async function tick() {
  try {
    S = await api.get('/api/state');
    const ej = S.jobs.find(j => j.kind === 'engine');
    if (ej && engineJobWas === 'running' && ej.state !== 'running') api.post('/api/os', { action: 'engine-refresh' }).catch(() => {});
    engineJobWas = ej ? ej.state : null;
    document.body.classList.toggle('dev', dev());
    session();
    renderStatus(); renderRail(); renderGuide(); renderPage();
  } catch (e) {
    $('#statusbar').innerHTML = `<span class="lamp"><i class="dot bad"></i><b>Studio's server is not answering.</b> Close this window and start Go2 Studio again.</span>`;
  }
}
async function tickWalks() {
  try { W = await api.get('/api/walks'); if (page === 'walks') renderWalks(); renderRail(); } catch (e) { /* next time */ }
}

/* ----------------------------------------------------------- navigation */
function go(p) {
  page = p; localStorage.setItem('go2.page', p);
  $$('.page').forEach(x => { x.hidden = x.id !== 'page-' + p; });
  $$('.nav button[data-page]').forEach(b => b.dataset.page === p ? b.setAttribute('aria-current', 'page') : b.removeAttribute('aria-current'));
  $('#content').scrollTop = 0;
  renderPage(true);
}
$$('.nav button[data-page]').forEach(b => b.addEventListener('click', () => go(b.dataset.page)));

function renderPage(force) {
  if (!S) return;
  if (page === 'connect') renderConnect();
  else if (page === 'record') renderRecord();
  else if (page === 'walks') renderWalks(force);
  else if (page === 'live') renderLive();
  else if (page === 'help' && force) renderHelp();
  else if (page === 'settings' && force) renderSettings();
  else if (page === 'settings' && $('#engine-box')) $('#engine-box').innerHTML = engineHTML();
}

/* ------------------------------------------------------------ status bar */
// A clock that is right is not news: it is shown only when it is wrong.
const quietClock = c => !(/clock/.test(c.headline) && c.ok);
function pfCheck(re) {
  const pf = R('preflight');
  return pf && pf.checks ? pf.checks.find(c => re.test(c.headline)) : null;
}
function lamp(state, name, value, goTo) {
  const cls = { on: 'on', warn: 'warn', bad: 'bad', off: '' }[state] || '';
  return `<button class="lamp" data-go="${goTo}"><i class="dot ${cls}"></i><b>${name}</b>${value ? `<span class="v">${esc(value)}</span>` : ''}</button>`;
}
function renderStatus() {
  const c = S.connect || {}, rb = S.robot;
  const clockC = pfCheck(/clock/), lidarC = pfCheck(/lidar/), roomC = pfCheck(/room|disk/);
  const power = (R('data') || {}).power;
  const reach = rb.reachable;
  let h = '';
  const rv = recView();
  if (reach) h += `<button class="robotpill ${rv === 'live' ? 'rec' : 'on'}" data-go="live"><i class="dot ${rv === 'live' ? 'rec' : 'on'}"></i>Robot connected<small>${esc(robotName())}</small></button>`;
  else if (rv === 'away') h += `<button class="robotpill rec" data-go="record"><i class="dot rec"></i>Out of reach, still recording<small>${Math.round(rb.last_seen_age || 0)} s</small></button>`;
  else h += `<button class="robotpill off" data-go="connect"><i class="dot ${c.link && c.link.ok ? 'warn' : 'bad'}"></i>Robot not connected</button>`;
  h += lamp(c.link ? (c.link.ok ? 'on' : 'bad') : 'off', 'Cable', '', 'connect');
  if (reach && clockC && !clockC.ok) h += lamp('bad', 'Robot clock', 'not set', 'record');
  if (reach && lidarC) h += lamp(lidarC.ok ? 'on' : 'bad', 'LiDAR', lidarC.ok ? 'answering' : 'silent', 'live');
  if (reach && power) h += lamp(power.soc > 30 ? 'on' : power.soc > 15 ? 'warn' : 'bad', 'Battery', power.soc + '%', 'live');
  if (reach && roomC) h += lamp(roomC.ok ? 'on' : 'warn', 'Robot disk', (roomC.detail.match(/[\d.]+ GB/) || [''])[0], 'live');
  const wd = S.walks_dir;
  if (wd && wd.free != null && wd.free < 20e9) h += lamp('warn', 'Laptop disk', fmtB(wd.free) + ' free', 'settings');
  if (dev()) h += `<span class="lamp"><i class="dot ${S.engine && S.engine.kind ? 'on' : 'bad'}"></i><b>Engine</b><span class="v">${esc(S.engine ? (S.engine.kind || 'none') : '...')}</span></span>`;
  h += '<span class="spacer"></span>';
  const job = S.jobs.find(j => j.state === 'running');
  if (job) {
    const verb = { download: 'Downloading', convert: 'Converting', map: 'Rebuilding', export: 'Copying', engine: 'Getting' }[job.kind];
    h += `<button class="jobchip" data-go="${job.kind === 'engine' ? 'settings' : 'walks'}"><span>${verb} <b>${esc(job.walk)}</b></span><span class="bar"><i style="width:${job.pct}%"></i></span><span class="mono">${Math.round(job.pct)}%</span></button>`;
  }
  $('#statusbar').innerHTML = h;
  $('#ver').innerHTML = `Studio ${esc(S.app.version)} (${esc(S.app.commit)})<br>robot ${esc(rb.base.replace('http://', ''))}`;
}
document.addEventListener('click', e => { const g = e.target.closest('[data-go]'); if (g) go(g.dataset.go); });

function renderRail() {
  if (!S) return;
  const c = S.connect;
  if (c) {
    const n = [c.link && c.link.ok, c.address && c.address.ok, c.robot && c.robot.ok, c.services && c.services.ok, c.lidar && c.lidar.ok].filter(Boolean).length;
    $('#nav-connect').textContent = n + '/5';
  }
  const pf = R('preflight');
  const rec = recView();
  $('#nav-record').textContent = rec === 'live' ? 'recording' : rec === 'away' ? 'recording' : S.robot.reachable && pf ? (pf.ready ? 'ready' : pf.blockers.length + ' to fix') : '';
  $('#nav-record').className = 'count' + (rec === 'live' || rec === 'away' ? ' live' : '');
  const running = S.jobs.filter(j => j.state === 'running' || j.state === 'queued').length;
  $('#nav-walks').textContent = running ? running + ' running' : (W.walks.length || '');
  $('#nav-walks').className = 'count' + (running ? ' live' : '');
}

/* --------------------------------------------------------------- connect */
function renderConnect() {
  const c = S.connect;
  if (!c) { $('#connect-steps').innerHTML = '<div class="muted">Checking...</div>'; return; }
  const clockC = pfCheck(/clock/);
  const steps = [
    ['Robot on, cable in', 'Ethernet from the robot\'s dock to this laptop.', c.link.ok, c.link.ok ? (c.link.adapter || 'link up') : 'no wired link', null],
    ['Laptop address', 'The wired adapter needs 192.168.123.99, on the robot\'s network.', c.address.ok,
      c.address.ok ? '192.168.123.99' : c.address.skip_as_source ? 'set, but unusable' : 'not set', c.address.ok ? null : addressHelp(c)],
    ['Robot answering', 'Main board .161 and onboard computer .18.', c.robot.ok,
      c.robot.ok ? 'both answering' : c.robot.starting ? 'onboard computer still booting' : 'no answer', c.robot.ok ? null : '<p class="muted" style="font-size:13.5px">It takes about a minute after power-on. Check the cable is in the dock\'s port, not the robot\'s body.</p>'],
    ['Robot programs running', 'The robot\'s dashboard, recorder and LiDAR relay. They start by themselves at power-on.', c.services.ok,
      c.services.ok ? 'answering' : 'not answering', c.services.ok ? null : '<p class="muted" style="font-size:13.5px">If the robot has been on for two minutes and this is still red, power-cycle it. The very first time, the programs have to be installed on the robot from the developer laptop (docs/dashboard.md).</p>'],
    ['LiDAR', 'The XT16 on top, spinning and answering.', c.lidar.ok, c.lidar.ok ? 'answering' : 'no answer', null],
  ];
  if (clockC && !clockC.ok) steps.push(['Robot clock', 'The robot has no clock battery and wakes up in 1970. Studio sets it by itself; if that failed, set it here.', false,
    'reads 1970', '<div style="margin-top:10px"><button class="btn primary" data-act="setclock">Set from this laptop</button></div>']);
  let first = true;
  $('#connect-steps').innerHTML = steps.map((s, i) => {
    const cls = s[2] ? 'ok' : first ? 'turn' : '';
    if (!s[2]) first = false;
    return `<div class="cstep ${cls}"><div class="cnum">${s[2] ? icon('check') : i + 1}</div><div><h3>${s[0]}</h3><p>${s[1]}</p>${!s[2] && s[4] ? s[4] : ''}</div><div class="state ${s[2] ? 'g' : 'a'}">${esc(s[3])}</div></div>`;
  }).join('');
}
function addressHelp(c) {
  const alias = (c.link && c.link.adapter) || 'Ethernet';
  const cmd = `New-NetIPAddress -InterfaceAlias '${alias}' -IPAddress 192.168.123.99 -PrefixLength 24`;
  return `<div class="admin">
    <div class="shield">${icon('shield')}Needs administrator rights</div>
    <div style="font-size:14.5px">Windows only lets an administrator add a network address. Studio adds it beside the normal one, so this port still works on other networks.</div>
    <div style="display:flex; gap:10px; margin-top:12px; flex-wrap:wrap; align-items:center">
      <button class="btn primary" data-act="addr-on">${icon('shield')}Set it for me</button>
      <span class="muted" style="font-size:13.5px">Windows asks "Do you want to allow this app to make changes?" Choose Yes.</span>
    </div>
    <details class="byhand"><summary>Do it by hand instead</summary>
      <ol class="steps">
        <li>Open the Start menu, type <b>PowerShell</b>, right-click it, <b>Run as administrator</b>.</li>
        <li>Paste this and press Enter:</li>
      </ol>
      <div class="cmd"><code>${esc(cmd)}</code><button data-copy="${esc(cmd)}">Copy</button></div>
      <div class="muted" style="font-size:13.5px">Or in Settings: Network and internet, Ethernet, IP assignment, Edit, Manual, IPv4 on: address <span class="mono g">192.168.123.99</span>, mask <span class="mono g">255.255.255.0</span>, no gateway. (This replaces the normal address, so undo it afterwards.)</div>
    </details>
    ${c.address.skip_as_source ? '<p class="a" style="font-size:13.5px">The address is there but marked "skip as source", so Windows will not use it to talk to the robot. The command above re-adds it properly after removing it.</p>' : ''}
  </div>`;
}

/* ---------------------------------------------------------------- record */
// What the Record page shows: ready | live | away | done
let rec = JSON.parse(localStorage.getItem('go2.rec') || 'null');   // {name, t0, markers, trail, notes}
let report = null;
function saveRec() { if (rec) localStorage.setItem('go2.rec', JSON.stringify(rec)); else localStorage.removeItem('go2.rec'); }
function recView() {
  if (!S) return 'ready';
  const r = R('recordings');
  const w = r && r.walk && r.walk.recording;
  if (S.robot.reachable && w && w.running) {
    // Started here, on the robot's own page, or on a phone: Studio follows it
    // either way, so that pulling the cable shows "out of reach, recording".
    if (!rec || rec.name !== w.name) { rec = { name: w.name, t0: Date.now() - (w.seconds || 0) * 1000, markers: [], trail: [], notes: null, adopted: true }; saveRec(); }
    return 'live';
  }
  if (rec && !S.robot.reachable) return 'away';
  if (rec && S.robot.reachable && !(w && w.running)) {
    // Stopped somewhere else, or it ran for a set time: show its report.
    report = { name: rec.name, verdict: r && r.walk ? r.walk.last_verdict : null };
    rec = null; saveRec();
  }
  return report ? 'done' : 'ready';
}

function renderRecord() {
  const view = recView();
  renderChecks(view);
  const right = $('#rec-right');
  if (view === 'ready') {
    if (!right.dataset.view || right.dataset.view !== 'ready') { right.innerHTML = setupHTML(); bindSetup(); }
    updateStart();
  } else if (view === 'live' || view === 'away') {
    if (right.dataset.view !== 'live') right.innerHTML = liveHTML();
    updateLive(view);
  } else {
    if (right.dataset.view !== 'done') right.innerHTML = reportHTML();
  }
  right.dataset.view = view === 'away' ? 'live' : view;
}

function renderChecks(view) {
  const pf = R('preflight');
  const card = $('#checks-card');
  card.style.opacity = view === 'live' || view === 'away' ? .55 : 1;
  if (!S.robot.reachable && !pf) {
    card.innerHTML = `<div class="empty"><b>The robot is not connected</b>The checks appear as soon as it answers. <button class="textlink" data-go="connect">Connect page</button></div>`;
    return;
  }
  const checks = ((pf && pf.checks) || []).filter(quietClock);
  const extra = [];
  const c = S.connect;
  if (c) extra.push({ ok: c.address.ok, headline: 'laptop address', detail: c.address.ok ? '192.168.123.99' : 'not set: Connect page, step 2' });
  const wd = S.walks_dir;
  extra.push({ ok: wd.exists && (wd.free == null || wd.free > 10e9), headline: 'room on this laptop', detail: (wd.free != null ? fmtB(wd.free) + ' free in ' : '') + wd.path });
  const all = checks.concat(extra);
  const ok = all.filter(x => x.ok).length;
  const blockers = checks.filter(x => x.blocking && !x.ok);
  const arc = 119.4 * (1 - ok / Math.max(1, all.length));
  card.innerHTML = `
    <div class="checks-sum">
      <svg class="ring" viewBox="0 0 46 46"><circle cx="23" cy="23" r="19" fill="none" stroke="#243429" stroke-width="4"/><circle cx="23" cy="23" r="19" fill="none" stroke="${blockers.length ? '#F2B84B' : '#4FE08A'}" stroke-width="4" stroke-linecap="round" stroke-dasharray="119.4" stroke-dashoffset="${arc}" transform="rotate(-90 23 23)"/></svg>
      <div><b>${ok === all.length ? 'All ' + all.length + ' ready' : ok + ' of ' + all.length + ' ready'}</b>
      <p>${blockers.length ? 'Before you can start: ' + blockers.map(b => b.headline).join(', ') + '.' : S.robot.reachable ? 'Stand clear and start when you are.' : 'Robot out of reach: these are its last answers.'}</p></div>
      <button class="textlink devnote" style="margin-left:auto" data-act="raw-preflight">{ } raw</button>
    </div>
    ${all.map(x => `
      <div class="check ${x.ok ? 'ok' : x.blocking ? 'bad' : 'warn'}">
        <i class="dot ${x.ok ? 'on' : x.blocking ? 'bad' : 'warn'}"></i>
        <div class="name">${esc(cap(x.headline))}${x.blocking ? '<small>needed to start</small>' : ''}</div>
        <div class="val"></div>
        <div class="detail">${esc(x.ok ? x.detail : shortFix(x))}${!x.ok && /clock/.test(x.headline) ? ` <button class="btn primary" style="margin-left:8px; padding:4px 10px" data-act="setclock">Set from this laptop</button>` : ''}</div>
      </div>`).join('')}
    <div class="devnote" style="margin-top:10px">GET ${esc(S.robot.base)}/api/preflight, age ${esc(Rage('preflight'))} s</div>`;
}
const cap = s => s ? s[0].toUpperCase() + s.slice(1) : s;
function shortFix(x) {
  const d = x.detail || '';
  if (/clock/.test(x.headline)) return 'Reads 1970. A walk recorded like that cannot be lined up.';
  const cut = d.indexOf('. ');
  return cut > 0 && cut < 160 ? d.slice(0, cut + 1) : d;
}

function setupHTML() {
  const last = JSON.parse(localStorage.getItem('go2.lastnotes') || '{}');
  const d = new Date();
  const stamp = d.getFullYear() + '-' + String(d.getMonth() + 1).padStart(2, '0') + '-' + String(d.getDate()).padStart(2, '0');
  return `<div class="card">
    <h2>This walk</h2>
    <div class="field"><label for="w-name">Name <span class="faint">(letters, digits, dash, underscore)</span></label><input class="input" id="w-name" value="${stamp}_walk-1" maxlength="64"></div>
    <div class="row">
      <div class="field"><label>Building</label><input class="input" id="w-building" value="${esc(last.building || '')}"></div>
      <div class="field"><label>Floor</label><input class="input" id="w-floor" value="${esc(last.floor || '')}"></div>
    </div>
    <div class="field"><label>Notes <span class="faint">(saved with the walk)</span></label><input class="input" id="w-notes" placeholder="route, people around, anything odd"></div>
    <div style="display:flex; align-items:center; gap:12px; margin:4px 0 6px">
      <button class="switch" role="switch" aria-checked="false" id="sw-depth"><i></i></button>
      <div><div style="font-size:14.5px">Record the depth camera <span class="faint" id="depth-state"></span></div><div class="faint" style="font-size:12.5px" id="depth-size">about 180 MB a minute; needed for depth on the map</div></div>
    </div>
    <div class="depth-opts" id="depth-opts">
      <div class="field"><label for="d-mode">Resolution and frame rate</label><select class="input" id="d-mode"></select></div>
      <label class="colourbox"><input type="checkbox" id="d-color">Colour stream <small>only needed for camera colour (an experiment); about a third more data</small></label>
      <div class="depth-apply" id="d-apply" hidden><span id="d-apply-t"></span><button class="btn primary" id="d-apply-go">Apply</button><button class="btn ghost" id="d-apply-undo">Undo</button></div>
    </div>
    <button class="bigstart" id="btn-start" disabled>${icon('play')}Start walk</button>
    <div class="why" id="why-start"></div>
    <ul class="tips">
      <li>Mark the start with tape and finish on the same spot.</li>
      <li>Walk and turn slowly; stay near walls, doors and furniture.</li>
      <li>1 to 5 minutes. <span id="minutes-left"></span></li>
    </ul></div>`;
}
/* The depth camera takes its mode when it starts. DOPT is the choice on this
 * page: it starts as what the camera runs (or last ran, per the robot), and a
 * different choice while it runs waits for Apply, which starts it again. Same
 * behaviour as the robot dashboard's Record page. */
const DOPT = { mode: null, color: false };
const depthModes = () => ((R('sensors') || {}).modes) || [{ id: '640x480x15', label: '640 x 480, 15 fps', proven: true }];
function depthParams(on) { return { action: on ? 'sensors-depth-on' : 'sensors-depth-off', mode: DOPT.mode || '640x480x15', color: DOPT.color }; }
function modeLabel(id) { const m = depthModes().find(x => x.id === id); return m ? m.label.replace(/ x /, ' × ') : (id || '').replace(/x(\d+)$/, ', $1 fps').replace('x', ' × '); }
function bindSetup() {
  $('#btn-start').onclick = startWalk;
  $('#sw-depth').onclick = async function () {
    const on = this.getAttribute('aria-checked') !== 'true';
    this.disabled = true;
    await act('/api/robot', { action: 'sensor', params: depthParams(on) },
      on ? 'Starting the depth camera at ' + modeLabel(DOPT.mode) + (DOPT.color ? ' with colour' : '') + ' (about 20 s).' : 'Stopping the depth camera.');
    this.disabled = false;
  };
  $('#d-mode').onchange = e => { DOPT.mode = e.target.value; updateStart(); };
  $('#d-color').onchange = e => { DOPT.color = e.target.checked; updateStart(); };
  $('#d-apply-undo').onclick = () => { const d = (R('sensors') || {}).depth || {}; DOPT.mode = d.mode; DOPT.color = !!d.color; updateStart(); };
  $('#d-apply-go').onclick = async () => {
    if (recView() === 'live') { toast('A walk is recording. Stop it first: restarting the camera would leave a gap in its depth.', { bad: true }); return; }
    await act('/api/robot', { action: 'sensor', params: depthParams(true) }, 'Restarting the depth camera at ' + modeLabel(DOPT.mode) + ' (about 20 s).');
  };
}
function updateDepthOpts(sens) {
  const sel = $('#d-mode'); if (!sel || !sens) return;
  const d = sens.depth || {}, want = sens.wanted || {}, busy = !!(sens.job && sens.job.running);
  if (!DOPT.mode) { DOPT.mode = d.on ? d.mode : (want.depth_mode || '640x480x15'); DOPT.color = d.on ? !!d.color : !!want.color; }
  const ms = depthModes(), key = ms.map(m => m.id).join();
  if (sel.dataset.v !== key) {
    sel.innerHTML = ms.map(m => `<option value="${esc(m.id)}">${esc(m.label.replace(/ x /, ' × '))}${m.proven ? '' : ' (not yet tested on this camera)'}</option>`).join('');
    sel.dataset.v = key;
  }
  if (document.activeElement !== sel) sel.value = DOPT.mode;
  $('#d-color').checked = DOPT.color;
  sel.disabled = $('#d-color').disabled = busy;
  const pending = d.on && !busy && (DOPT.mode !== d.mode || DOPT.color !== !!d.color);
  $('#d-apply').hidden = !pending;
  if (pending) $('#d-apply-t').textContent = `Running at ${modeLabel(d.mode)}${d.color ? ' with colour' : ''}. Apply restarts it (about 20 s).`;
  // a rough size, so a bigger mode is a visible choice
  const [w, h, f] = (DOPT.mode || '640x480x15').split('x').map(Number), k = (w * h) / (640 * 480) * f / 15;
  $('#depth-size').textContent = `about ${Math.round(180 * k * (DOPT.color ? 1.33 : 1))} MB a minute; needed for depth on the map` + (busy ? ` · ${esc(sens.job.title)}...` : '');
}
function updateStart() {
  const pf = R('preflight');
  const btn = $('#btn-start'); if (!btn) return;
  const reach = S.robot.reachable;
  btn.disabled = !reach || !pf || !pf.ready;
  $('#why-start').innerHTML = !reach ? 'The robot is not connected.' : !pf ? 'Waiting for the robot\'s checks.' :
    pf.ready ? 'Everything is ready.' : 'First: ' + esc(pf.blockers.join(', ')) + '. <button class="textlink" data-act="force-start">Start anyway</button>';
  const sens = R('sensors');
  const depth = sens && sens.depth;
  const sw = $('#sw-depth');
  if (sw && depth && !sw.disabled) sw.setAttribute('aria-checked', depth.on ? 'true' : 'false');
  const ds = $('#depth-state');
  if (ds) ds.textContent = depth ? '(' + (depth.on ? modeLabel(depth.mode) + (depth.color ? ', colour' : '') + (depth.fps ? ', ' + Math.round(depth.fps) + ' fps arriving' : '') : 'off') + ')' : '';
  updateDepthOpts(sens);
  const roomC = pfCheck(/room/);
  const ml = $('#minutes-left');
  if (ml && roomC) ml.textContent = (roomC.detail.match(/about [\d,]+ minutes/) || [''])[0].replace('about', 'About').replace('minutes', 'minutes fit on the robot now.');
}
async function startWalk(force) {
  const name = $('#w-name').value.trim();
  if (!/^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$/.test(name)) { toast('Use letters, digits, dash and underscore for the name, no spaces.', { bad: true }); return; }
  const notes = { building: $('#w-building').value, floor: $('#w-floor').value, notes: $('#w-notes').value };
  localStorage.setItem('go2.lastnotes', JSON.stringify(notes));
  $('#btn-start').disabled = true;
  $('#why-start').textContent = 'Starting the recorder on the robot...';
  const r = await act('/api/robot', { action: 'walk-start', name, notes, force: force === true });
  if (!r) { updateStart(); return; }
  rec = { name, t0: Date.now(), markers: [], trail: [], notes }; saveRec();
  report = null;
  renderRecord();
}
document.addEventListener('click', e => { if (e.target.closest('[data-act="force-start"]')) confirmBox('Start anyway?', 'The checks say this walk will probably not be usable. Start only if you know why a check is wrong.', 'Start anyway', () => startWalk(true)); });

function liveHTML() {
  return `<div class="card">
    <div class="live-top">
      <div><div class="recbadge"><i class="dot rec"></i><span id="rec-label">RECORDING</span></div><div class="timer" id="timer">00:00</div><div class="muted" id="rec-name" style="font-size:13.5px"></div></div>
      <button class="btn stop" data-act="stop">${icon('stop')}Stop walk</button>
    </div>
    <div class="banner info" id="unplug-hint">${icon('check', 'g')}<div id="unplug-text">Waiting for the counters to climb...</div></div>
    <div class="banner warn" id="away" hidden>${icon('plug', 'a')}<div id="away-text"></div></div>
    <div id="live-body">
      <div class="counters" id="counters"></div>
      <div style="margin-top:14px" class="trail">
        <svg viewBox="0 0 400 250" id="trail-svg"></svg>
        <div class="home"><b id="home-d">-</b><span>from where you started</span></div>
        <div class="legend" id="trail-legend">Top view from the robot's odometry.</div>
      </div>
      <div class="markers">
        <button class="btn" data-act="marker">${icon('flag')}Drop a marker</button>
        <span class="muted" id="mk-list">No markers yet. Drop one at stairs, doors, anything worth finding later.</span>
        <span class="muted" style="margin-left:auto" id="rec-size"></span>
      </div>
    </div></div>`;
}
function spark(ok) { return `<svg viewBox="0 0 100 6" preserveAspectRatio="none" style="height:6px"><rect width="100" height="6" rx="3" fill="${ok ? '#1E5A38' : '#5a2a25'}"/></svg>`; }
function updateLive(view) {
  const r = R('recordings');
  const st = (r && r.walk && r.walk.recording) || {};
  const away = view === 'away';
  const elapsed = st.seconds != null && !away ? st.seconds : rec ? (Date.now() - rec.t0) / 1000 : 0;
  $('#timer').textContent = clock(elapsed);
  $('#rec-name').textContent = (st.name || (rec && rec.name) || '');
  $('#live-body').classList.toggle('stale', away);
  $('#away').hidden = !away;
  $('#unplug-hint').hidden = away;
  if (away) $('#away-text').innerHTML = `<b>Robot out of reach, as expected.</b> It is still recording on its own; last seen ${Math.round(S.robot.last_seen_age || 0)} s ago. Plug the cable back in to see it and stop.`;
  const health = st.health || [];
  const allOk = health.length && health.every(h => h.ok);
  $('#unplug-text').innerHTML = allOk ? 'Everything is arriving. <b>You can unplug the cable now</b> and walk. Plug it back in to stop.' :
    health.length ? 'Not everything is arriving yet: ' + esc(health.filter(h => !h.ok).map(h => h.name + ': ' + h.note).join('; ')) : 'Waiting for the counters to climb...';
  $('#unplug-hint').className = 'banner ' + (allOk || !health.length ? 'info' : 'bad');
  if (health.length) {
    $('#counters').innerHTML = health.map(h => `<div class="counter"><div class="lbl"><span>${esc(cap(h.name))}</span><span class="${h.ok ? 'g' : 'r'}">${Math.round(h.rate)} ${esc(h.unit)}</span></div><div class="num">${Number(h.count).toLocaleString()}</div>${spark(h.ok)}</div>`).join('');
  }
  if (st.bytes_on_disk) $('#rec-size').textContent = fmtB(st.bytes_on_disk) + ' so far' + (st.minutes_left != null ? ', room for ' + Math.round(st.minutes_left) + ' min more' : '');
  // Trail from the robot's own position, only while it answers.
  const m = (R('data') || {}).motion;
  if (rec && m && m.position && !away) {
    const [x, y] = m.position, t = rec.trail, last = t[t.length - 1];
    if (!last || Math.hypot(x - last[0], y - last[1]) > 0.05) { t.push([x, y]); if (t.length > 4000) t.splice(1, 1); saveRec(); }
  }
  drawTrail();
  if (rec) $('#mk-list').innerHTML = rec.markers.length ? rec.markers.length + ' marker' + (rec.markers.length > 1 ? 's' : '') + ': ' + rec.markers.map(k => `<span class="a">${esc(k.label)}</span> at ${clock(k.t)}`).join(', ') : $('#mk-list').innerHTML;
}
function drawTrail() {
  const t = rec ? rec.trail : [];
  const svg = $('#trail-svg'); if (!svg) return;
  let g = '<defs><pattern id="grid" width="25" height="25" patternUnits="userSpaceOnUse"><path d="M25 0H0V25" fill="none" stroke="#111a14" stroke-width="1"/></pattern></defs><rect width="400" height="250" fill="url(#grid)"/>';
  if (t.length > 1) {
    const xs = t.map(p => p[0]), ys = t.map(p => p[1]);
    const minx = Math.min(...xs), maxx = Math.max(...xs), miny = Math.min(...ys), maxy = Math.max(...ys);
    const span = Math.max(maxx - minx, maxy - miny, 4);
    const sc = Math.min(360 / span, 210 / span);
    const cx = (minx + maxx) / 2, cy = (miny + maxy) / 2;
    const P = p => [200 + (p[0] - cx) * sc, 125 - (p[1] - cy) * sc];
    const pts = t.map(p => P(p).map(v => v.toFixed(1)).join(',')).join(' ');
    const s = P(t[0]), e = P(t[t.length - 1]);
    g += `<polyline points="${pts}" fill="none" stroke="#4FE08A" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round" opacity=".9"/>`;
    g += `<circle cx="${s[0]}" cy="${s[1]}" r="7" fill="none" stroke="#E3F2E8" stroke-width="2"/><circle cx="${e[0]}" cy="${e[1]}" r="6" fill="#4FE08A"/>`;
    g += `<path d="M${e[0]} ${e[1]} L${s[0]} ${s[1]}" stroke="#F2B84B" stroke-dasharray="3 3"/>`;
    const d = Math.hypot(t[0][0] - t[t.length - 1][0], t[0][1] - t[t.length - 1][1]);
    let walked = 0; for (let i = 1; i < t.length; i++) walked += Math.hypot(t[i][0] - t[i - 1][0], t[i][1] - t[i - 1][1]);
    $('#home-d').textContent = d.toFixed(1) + ' m';
    $('#trail-legend').textContent = 'Top view from the robot\'s odometry. ' + walked.toFixed(0) + ' m walked. Grid ' + (25 / sc).toFixed(1) + ' m.';
  }
  svg.innerHTML = g;
}
async function stopWalk() {
  const btn = $('[data-act="stop"]'); if (btn) { btn.disabled = true; btn.lastChild.textContent = 'Stopping...'; }
  const r = await act('/api/robot', { action: 'walk-stop' });
  if (!r) { if (btn) { btn.disabled = false; btn.lastChild.textContent = 'Stop walk'; } return; }
  report = { name: rec ? rec.name : '', verdict: r.verdict, warning: r.warning, notes: rec ? Object.assign({}, rec.notes, { markers: rec.markers }) : null };
  if (rec) await api.post('/api/walks', { action: 'notes-pending', name: rec.name, notes: report.notes }).catch(() => {});
  rec = null; saveRec();
  $('#rec-right').dataset.view = '';
  renderRecord();
}
function reportHTML() {
  const v = report && report.verdict;
  const vv = v && v.verdict;
  const good = vv && vv.ok;
  const name = (v && v.name) || (report && report.name) || 'the walk';
  return `<div class="card">
    <div style="display:flex; align-items:center; gap:12px">${icon(good ? 'check' : 'warn', good ? 'g' : 'a')}
      <div><h2 style="margin:0">${esc(name)} ${good ? 'is good to map' : vv ? 'has problems' : 'stopped'}</h2>
      <div class="muted" style="font-size:14px">${v ? fmtDur(v.seconds) + ', ' + fmtB(v.bytes) + ' on the robot. ' : ''}${vv ? vv.passed + ' of ' + vv.total + ' checks passed.' : ''}</div></div></div>
    ${report && report.warning ? `<div class="banner warn">${icon('warn', 'a')}<div>${esc(report.warning)}</div></div>` : ''}
    <div style="margin-top:14px">${vv ? vv.checks.filter(quietClock).map(c => `<div class="report-row">${icon(c.ok ? 'check' : 'x', c.ok ? 'g' : 'r')}${esc(cap(c.headline))}<span class="val">${esc(c.detail).slice(0, 90)}</span></div>`).join('') : ''}</div>
    <div style="display:flex; gap:10px; margin-top:18px; flex-wrap:wrap">
      <button class="btn primary" data-act="dl-convert" data-name="${esc(name)}">${icon('down')}Download and make the map</button>
      <button class="btn" data-act="dl-only" data-name="${esc(name)}">${icon('down')}Download only</button>
      <button class="btn ghost" data-act="again">Record another</button>
    </div>
    <div class="muted" style="font-size:13.5px; margin-top:10px">Downloads over the cable at about 50 MB/s. You can keep recording while a walk converts.</div>
  </div>`;
}

/* ----------------------------------------------------------------- walks */
let filter = 'all', search = '';
const STAGES = [['robot', 'On the robot'], ['downloaded', 'Downloaded'], ['bag', 'Bag'], ['map', 'Map'], ['dense', 'Clean cloud']];
function walkJob(name) { return S.jobs.find(j => j.walk === name && j.kind !== 'export'); }
function stageStates(w) {
  const st = {};
  const s = w.stages || {};
  st.robot = { c: 'done', r: w.on_robot === false ? 'removed' : '' };
  st.downloaded = { c: s.downloaded ? 'done' : '', r: '' };
  st.bag = { c: s.bag ? 'done' : s.bag_broken ? 'fail' : '', r: s.bag_broken ? 'cut off' : '' };
  st.map = { c: w.can_map === false ? 'na' : s.map ? 'done' : '', r: w.can_map === false ? 'n/a' : '' };
  st.dense = { c: w.can_map === false ? 'na' : s.dense ? 'done' : '', r: w.can_map === false ? 'n/a' : '' };
  const j = walkJob(w.name);
  if (j && (j.state === 'running' || j.state === 'queued' || j.state === 'failed')) {
    const map = { download: 'downloaded', unpack: 'downloaded', bag: 'bag', map: 'map', dense: 'dense' };
    const key = map[j.stage];
    if (j.state === 'queued') { st[map[j.stages[0]]] = { c: 'run', r: 'queued', p: 0 }; }
    else if (j.state === 'running') {
      const order = j.stages.map(x => map[x]);
      for (const k of order) { if (k === key) break; st[k] = { c: 'done', r: '' }; }
      const pct = j.kind === 'download' ? (j.stage === 'download' ? j.stage_pct * 0.9 : 90 + j.stage_pct * 0.1) : j.stage_pct;
      st[key] = { c: 'run', p: pct, r: Math.round(pct) + '%' + (j.eta ? ' · ' + fmtEta(j.eta) : '') };
    } else if (j.state === 'failed' && !(w.stages || {})[key]) st[key] = { c: 'fail', r: 'failed' };
  }
  return st;
}
function renderWalks() {
  if (!S) return;
  const wd = S.walks_dir;
  const eng = S.engine;
  $('#engine-banner').innerHTML = eng && !eng.kind ? `<div class="banner warn" style="margin:0 0 14px">${icon('warn', 'a')}<div><b>Making maps needs one more thing:</b> ${esc(eng.detail)}. <button class="textlink" data-go="settings">Set it up in Settings</button></div></div>` : '';
  $('#folderbar').innerHTML = `${icon('folder', 'muted')}<span>Walks folder</span><span class="path">${esc(wd.path)}</span>
    <button class="textlink" data-act="show-walks">Open</button><button class="textlink" data-act="pick-walks">Change</button>
    ${wd.onedrive ? '<span class="a" style="font-size:13.5px">inside OneDrive: multi-GB files will sync slowly. Choose another folder.</span>' : ''}
    ${!wd.exists ? '<span class="a" style="font-size:13.5px">does not exist yet; it is made on the first download</span>' : ''}
    <span style="margin-left:auto" class="muted">${wd.free != null ? fmtB(wd.free) + ' free of ' + fmtB(wd.total) : ''}</span>
    ${wd.total ? `<span class="diskbar"><i style="width:${Math.round(100 * (1 - wd.free / wd.total))}%"></i></span>` : ''}
    ${W.robot_reachable ? '' : '<span class="muted" style="font-size:13px">robot not connected: showing this laptop only</span>'}`;
  const ws = W.walks;
  const isReady = w => w.stages && w.stages.dense;
  const needs = w => { const j = walkJob(w.name); return (j && j.state === 'failed') || w.can_map === false || (w.verdict && w.verdict.ok === false) || (w.stages && w.stages.bag_broken); };
  const counts = { all: ws.length, robot: ws.filter(w => w.stages && !w.stages.downloaded).length, ready: ws.filter(isReady).length, attention: ws.filter(needs).length };
  if (!$('#filters').dataset.built) {
    $('#filters').dataset.built = 1;
    $('#filters').innerHTML = `<span id="chips"></span><input class="search" id="walk-search" placeholder="Search name, building, notes">`;
    $('#walk-search').oninput = e => { search = e.target.value.toLowerCase(); renderWalks(); };
  }
  $('#chips').innerHTML = [['all', 'All'], ['robot', 'On the robot only'], ['ready', 'Ready to open'], ['attention', 'Needs attention']]
    .map(([k, l]) => `<button class="chip" data-filter="${k}" aria-pressed="${filter === k}">${l} <span class="mono">${counts[k]}</span></button> `).join('');
  let list = ws.filter(w => filter === 'all' || (filter === 'robot' && w.stages && !w.stages.downloaded) || (filter === 'ready' && isReady(w)) || (filter === 'attention' && needs(w)));
  if (search) list = list.filter(w => (w.name + ' ' + JSON.stringify(w.notes || {})).toLowerCase().includes(search));
  if (!ws.length) {
    $('#walklist').innerHTML = `<div class="empty"><b>No walks yet</b>Record one on the Record page, or put walk folders in ${esc(wd.path)}.</div>`;
    return;
  }
  // Keep open menus open across the 1-second redraws.
  if ($('.menu-list:not([hidden])')) return;
  $('#walklist').innerHTML = list.map(walkCard).join('') || '<div class="empty"><b>Nothing matches</b>Try another filter.</div>';
}
document.addEventListener('click', e => { const f = e.target.closest('[data-filter]'); if (f) { filter = f.dataset.filter; renderWalks(); } });

// Camera colour is an experiment that adds 5-10 minutes, so it is a choice
// made beside the button, remembered in Settings.
const colourBox = () => `<label class="colourbox" title="Colours the clean cloud from the camera. An experiment: adds 5-10 minutes and the result is not reliable yet."><input type="checkbox" data-act="colour" ${S.settings.colour ? 'checked' : ''}>Camera colour <small>experiment, +5-10 min</small></label>`;
function walkCard(w) {
  const st = stageStates(w);
  const j = walkJob(w.name);
  const busy = j && (j.state === 'running' || j.state === 'queued');
  const n = esc(w.name);
  const tags = [];
  if (w.clock_ok === false) tags.push(['r', 'clock not set']);
  if (w.can_map === false) tags.push(['r', 'no odometry']); else if (w.can_map) tags.push(['g', 'odometry']);
  if (w.depth) tags.push(['g', 'depth']);
  if (w.verdict && w.verdict.total) tags.push([w.verdict.ok ? 'g' : 'a', w.verdict.passed + '/' + w.verdict.total + ' checks']);
  const notes = w.notes || {};
  const where = [notes.building, notes.floor ? 'floor ' + notes.floor : ''].filter(Boolean).join(', ');
  let note = '';
  if (w.track) note = `Track ${w.track.length_m} m, ends ${w.track.closure_m} m from its start${w.points ? '. ' + (w.points / 1e6).toFixed(2) + ' M points in the clean cloud' : ''}.`;
  if (w.can_map === false) note = 'Recorded without odometry (robot lying down, or before it was recorded), so it cannot become a map.';
  const err = j && j.state === 'failed' ? `<div class="err-line">${icon('warn', 'r')} ${esc(j.kind === 'download' ? 'Download' : 'Conversion')} failed: ${esc(j.error || 'see the log')} <button class="textlink" data-act="joblog" data-job="${j.id}">Log</button></div>` : '';

  let main = '';
  const downloaded = w.stages && w.stages.downloaded;
  if (busy) main = `<button class="btn danger" data-act="cancel" data-job="${j.id}">${icon('x')}Cancel</button><span class="muted" style="font-size:13px">${esc(j.note || '')}${j.kind !== 'download' ? '. You can close Studio; it keeps going.' : ''}</span>`;
  else if (!downloaded) main = W.robot_reachable ? `<button class="btn primary" data-act="dl-convert" data-name="${n}" data-bytes="${w.bytes || 0}">${icon('down')}Download and make the map</button><button class="btn" data-act="dl-only" data-name="${n}" data-bytes="${w.bytes || 0}">Download only</button><span class="muted" style="font-size:13px">${fmtB(w.bytes)}</span>${colourBox()}` : '<span class="muted" style="font-size:13.5px">Connect the robot to download it.</span>';
  else if (w.stages.dense || w.stages.map) main = `<button class="btn primary" data-act="open" data-name="${n}" data-file="${n === '' ? '' : 'map3d/' + n + '_map/' + n + '_map_0.mcap'}" data-with="foxglove">${icon('cube')}Map in Foxglove</button>` +
    // The coloured cloud is the better one to look at, when the walk has colour.
    (w.stages.colour ? `<button class="btn" data-act="open" data-name="${n}" data-file="map3d/${n}_colour.ply" data-with="cloudcompare" title="The clean cloud in the camera's colours">${icon('cloud')}Colour cloud</button>`
      : w.stages.dense ? `<button class="btn" data-act="open" data-name="${n}" data-file="map3d/${n}_dense.ply" data-with="cloudcompare">${icon('cloud')}Clean cloud</button>` : '') +
    ((w.plans || []).length ? `<button class="btn" data-act="detail" data-name="${n}" data-to="plans" title="The floor plans, in the walk's details">${icon('map')}Floor plan${w.plans.length > 1 ? 's (' + w.plans.length + ')' : ''}</button>` : '') +
    (w.stages.mesh ? `<button class="btn" data-act="open" data-name="${n}" data-file="map3d/${n}_mesh.ply" data-with="cloudcompare" title="Surfaces as a coloured triangle mesh">${icon('cube')}Mesh</button>` : '');
  else if (w.can_map === false) main = w.stages.bag ? `<button class="btn" data-act="open" data-name="${n}" data-file="bag/bag_0.mcap" data-with="foxglove">${icon('cube')}Open bag in Foxglove</button>` : '';
  else main = `<button class="btn primary" data-act="convert" data-name="${n}">Make the map</button><span class="muted" style="font-size:13px">about ${fmtDur((w.seconds || 60) * (S.settings.colour ? 2.5 : 1.5))}</span>${colourBox()}`;

  const menu = `<div class="menu"><button class="btn ghost" data-act="menu" aria-label="More">${icon('more')}</button>
    <div class="menu-list" hidden>
      ${downloaded ? `<button data-act="detail" data-name="${n}">Details and files</button>
      <button data-act="convert" data-name="${n}" ${busy ? 'disabled' : ''}>Make the map again</button>
      <button data-act="remap" data-name="${n}" ${busy || !w.stages.bag ? 'disabled' : ''}>Rebuild map only (keep the bag)</button>
      <button data-act="colour-menu">Camera colour: ${S.settings.colour ? 'on' : 'off'} (click to turn ${S.settings.colour ? 'off' : 'on'})</button><hr>
      <button class="r" data-act="del-local" data-name="${n}">Delete from this laptop</button>` : ''}
      ${w.on_robot ? `<button class="r" data-act="del-robot" data-name="${n}" data-safe="${downloaded ? 1 : 0}">Delete from the robot${downloaded ? ' (copy is safe here)' : ''}</button>` : ''}
    </div></div>`;

  return `<article class="walk ${busy ? 'busy' : ''}">
    <div class="walk-top">
      <div><div class="walk-title" ${downloaded ? `data-act="detail" data-name="${n}"` : ''}>${n}</div>
        <div class="walk-meta"><span>${fmtDate(w.started)}</span><span>${fmtDur(w.seconds)}</span><span>${fmtB(w.bytes)}</span>${where ? `<span>${esc(where)}</span>` : ''}</div></div>
      <div class="walk-tags">${tags.map(t => `<span class="tag ${t[0]}">${esc(t[1])}</span>`).join('')}</div>
    </div>
    <div class="pipe">${STAGES.map(([k, label]) => { const s = st[k] || {}; return `<div class="stage ${s.c || ''}"><div class="bar"><i style="${s.c === 'run' ? 'width:' + (s.p || 0) + '%' : ''}"></i></div><div class="lbl"><b>${label}</b><span>${esc(s.r || '')}</span></div></div>`; }).join('')}</div>
    ${note ? `<div class="note ${w.can_map === false ? 'a' : 'muted'}" style="margin:-4px 0 10px">${esc(note)}</div>` : ''}
    ${err}
    <div class="walk-actions">${main}
      <div class="right">
        ${downloaded && j ? `<button class="btn ghost devnote-btn" data-act="joblog" data-job="${j.id}">${icon('code')}Log</button>` : ''}
        ${downloaded ? `<button class="btn ghost" data-act="show" data-name="${n}">${icon('folder')}Show in folder</button>
        <button class="btn ghost" data-act="saveas" data-name="${n}">${icon('save')}Save a copy</button>` : ''}
        ${menu}
      </div></div>
  </article>`;
}

/* ------------------------------------------------------- detail and copy */
async function openDetail(name, to) {
  let w;
  try { w = await api.post('/api/walks', { action: 'detail', name }); } catch (e) { toast(e.message, { bad: true }); return; }
  const n = esc(w.name);
  const notes = w.notes || {};
  const jobsFor = S.jobs.filter(j => j.walk === w.name);
  $('#drawer').innerHTML = `<button class="btn ghost x" data-act="close">${icon('x')}</button>
    <div class="muted" style="font-size:13px">Walk</div>
    <h1 style="font-size:28px">${n}</h1>
    <div class="walk-meta" style="margin-top:6px"><span>${fmtDate(w.started)}</span><span>${fmtDur(w.seconds)}</span><span>${fmtB(w.bytes)} on this laptop</span></div>
    ${w.track || w.points ? `<div class="sect"><h3>Map</h3><div class="metrics">
      ${w.track ? `<div class="metric"><b>${w.track.length_m} m</b><span>walked</span></div><div class="metric"><b class="g">${w.track.closure_m} m</b><span>end to start</span></div>` : ''}
      ${w.points ? `<div class="metric"><b>${(w.points / 1e6).toFixed(2)} M</b><span>points in the clean cloud</span></div>` : ''}</div></div>` : ''}
    ${(w.plans || []).length ? `<div class="sect" id="d-plans"><h3>Floor plans</h3><div class="plans">
      ${w.plans.map(p => `<figure class="plan"><img src="/api/walk-image?name=${encodeURIComponent(w.name)}&file=${encodeURIComponent(p.rel)}&v=${p.bytes}" alt="Floor plan, level ${p.level}" data-act="open" data-name="${n}" data-file="${esc(p.rel)}" title="Open full size">
        <figcaption><b>Level ${p.level}</b>${p.height != null ? ` floor at ${p.height >= 0 ? '+' : ''}${p.height.toFixed(2)} m` : ''}${p.floor_m2 ? `, ${Math.round(p.floor_m2)} m² floor` : ''}
          <span><button class="btn ghost" data-act="open" data-name="${n}" data-file="${esc(p.rel)}">Open</button><button class="btn ghost" data-act="show" data-name="${n}" data-file="${esc(p.rel)}">Show</button></span></figcaption></figure>`).join('')}
      </div><div class="muted" style="font-size:12.5px; margin-top:6px">Beside each picture in the folder: a .pgm and .yaml of the same plan, the map format ROS navigation reads.</div></div>` : ''}
    <div class="sect"><h3>Files</h3><table class="files">
      ${w.files.map(f => `<tr><td><span class="fn">${esc(f.rel)}</span><small>${esc(f.what)}</small></td><td class="sz">${fmtB(f.bytes)}</td><td class="act">
        <button class="btn ghost" data-act="open" data-name="${n}" data-file="${esc(f.rel)}" data-with="${f.rel.endsWith('.ply') ? 'cloudcompare' : f.rel.endsWith('.mcap') ? 'foxglove' : ''}">Open</button>
        <button class="btn ghost" data-act="show" data-name="${n}" data-file="${esc(f.rel)}">Show</button></td></tr>`).join('')}
      <tr><td><span class="fn">${esc(w.raw_files.join(', '))}</span><small>The recording as it came off the robot</small></td><td class="sz">${fmtB(w.raw_bytes)}</td><td class="act"><button class="btn ghost" data-act="show" data-name="${n}" data-file="lidar.pcap">Show</button></td></tr>
    </table></div>
    ${w.verdict && w.verdict.checks ? `<div class="sect"><h3>Recording checks</h3>${w.verdict.checks.filter(quietClock).map(c => `<div class="report-row">${icon(c.ok ? 'check' : 'x', c.ok ? 'g' : 'r')}${esc(cap(c.headline))}<span class="val">${esc(String(c.detail).slice(0, 70))}</span></div>`).join('')}</div>` : ''}
    <div class="sect"><h3>About this walk</h3>
      <div class="row"><div class="field"><label>Building</label><input class="input" id="d-building" value="${esc(notes.building || '')}"></div><div class="field"><label>Floor</label><input class="input" id="d-floor" value="${esc(notes.floor || '')}"></div></div>
      <div class="field"><label>Notes</label><textarea class="input" rows="2" id="d-notes">${esc(notes.notes || '')}</textarea></div>
      ${(notes.markers || []).length ? `<div class="muted" style="font-size:13.5px">Markers: ${notes.markers.map(m => esc(m.label) + ' at ' + clock(m.t)).join(', ')}</div>` : ''}
      <button class="btn" data-act="save-notes" data-name="${n}">Save</button></div>
    <div class="sect devnote"><h3 style="color:var(--cyan)">Developer</h3>
      <dl class="kv mono" style="font-size:12.5px"><dt>folder</dt><dd>${esc(w.path)}</dd>
      ${jobsFor.map(j => `<dt>${esc(j.kind)}</dt><dd>${esc(j.state)} ${j.finished ? 'in ' + Math.round(j.finished - j.started) + ' s' : ''} via ${esc(j.engine || '-')} <button class="textlink" data-act="joblog" data-job="${j.id}">log</button></dd>`).join('')}</dl>
      <div style="margin-top:8px; display:flex; gap:8px"><button class="btn" data-act="show" data-name="${n}" data-file="studio-convert.log">Last conversion log file</button><button class="btn" data-act="open" data-name="${n}" data-file="session.json">session.json</button></div></div>`;
  openOverlay('#drawer');
  if (to && $('#d-' + to)) $('#d-' + to).scrollIntoView({ block: 'start' });
}

async function openSaveAs(name) {
  let w;
  try { w = await api.post('/api/walks', { action: 'detail', name }); } catch (e) { toast(e.message, { bad: true }); return; }
  const opts = w.files.map(f => ({ key: f.rel, label: f.what, bytes: f.bytes, on: ['map', 'dense', 'colour', 'mesh'].includes(f.key) }));
  (w.plans || []).forEach(p => opts.push({ key: p.rel, label: 'Floor plan, level ' + p.level, bytes: p.bytes, on: true }));
  opts.push({ key: '__raw', label: 'Raw recording (enough to convert again anywhere)', bytes: w.raw_bytes, on: false });
  const dest = (S.home || 'C:\\') + '\\Desktop';
  $('#modal').innerHTML = `<button class="btn ghost x" data-act="close">${icon('x')}</button>
    <h2 style="font-size:22px">Save a copy of ${esc(w.name)}</h2>
    <p class="muted" style="margin:0 0 10px; font-size:14.5px">Pick what to copy and where. The walk also stays in the walks folder.</p>
    ${opts.map(o => `<label class="opt"><input type="checkbox" data-key="${esc(o.key)}" data-bytes="${o.bytes}" ${o.on ? 'checked' : ''}><div>${esc(o.label)}<small>${esc(o.key === '__raw' ? w.raw_files.join(', ') : o.key)}</small></div><span class="sz">${fmtB(o.bytes)}</span></label>`).join('')}
    <div class="field" style="margin-top:16px"><label>Save to</label>
      <div class="pathpick"><input class="input" id="sa-dest" value="${esc(dest)}"><button class="btn" data-act="sa-browse">${icon('folder')}Browse...</button></div></div>
    <div style="display:flex; align-items:center; gap:12px; margin-top:8px">
      <label style="display:flex; gap:8px; align-items:center; font-size:14px"><input type="checkbox" id="sa-zip" style="accent-color:var(--green)"> as one .zip</label>
      <span class="muted" style="margin-left:auto; font-size:13.5px" id="sa-total"></span></div>
    <div style="display:flex; justify-content:flex-end; gap:10px; margin-top:18px">
      <button class="btn ghost" data-act="close">Cancel</button><button class="btn primary" data-act="sa-go" data-name="${esc(w.name)}" data-raw="${esc(w.raw_files.join('|'))}">${icon('save')}Save copy</button></div>`;
  const total = () => { const b = $$('#modal input[data-key]:checked').reduce((a, x) => a + +x.dataset.bytes, 0); $('#sa-total').textContent = fmtB(b) + ' selected'; };
  $$('#modal input[data-key]').forEach(x => x.onchange = total); total();
  openOverlay('#modal');
}

/* ----------------------------------------------------------------- robot */
function renderRobot() {
  const d = R('data'), sens = R('sensors');
  const body = $('#robot-body');
  if (!d) { body.innerHTML = `<div class="empty"><b>The robot is not connected</b>This page fills in as soon as it answers. <button class="textlink" data-go="connect">Connect page</button></div>`; return; }
  const p = d.power || {}, joints = d.joints || [], m = d.motion || {};
  const hot = joints.reduce((a, j) => (j.temp > (a ? a.temp : -1) ? j : a), null);
  const stale = !S.robot.reachable ? `<div class="stale-note">Out of reach: last answer ${Math.round(S.robot.last_seen_age || 0)} s ago.</div>` : '';
  const roomC = pfCheck(/room/);
  body.innerHTML = `${stale}<div class="tiles">
    <div class="tile"><div class="t"><span>Battery</span><span class="${p.soc > 30 ? 'g' : 'a'}">${p.soc > 30 ? 'OK' : 'LOW'}</span></div><div class="big">${p.soc ?? '?'}%</div><div class="sub">${p.volts ?? '?'} V, ${p.amps ?? '?'} A, ${p.temp ?? '?'} °C</div><div class="gauge"><i style="width:${p.soc || 0}%"></i></div></div>
    <div class="tile"><div class="t"><span>Hottest motor</span><span class="${hot && hot.temp > 70 ? 'a' : 'g'}">${hot && hot.temp > 70 ? 'HOT' : 'OK'}</span></div><div class="big">${hot ? hot.temp : '?'} °C</div><div class="sub">${hot ? esc(hot.name) : ''}; warn at 70</div><div class="gauge"><i style="width:${hot ? Math.min(100, hot.temp) : 0}%"></i></div></div>
    <div class="tile"><div class="t"><span>Motion</span><span class="g">${esc(m.mode_name || '')}</span></div><div class="big">${m.body_height != null ? m.body_height.toFixed(2) + ' m' : '?'}</div><div class="sub">body height; ${esc(m.gait || '')}</div></div>
    <div class="tile"><div class="t"><span>Robot disk</span><span class="${roomC && roomC.ok ? 'g' : 'a'}">${roomC ? (roomC.ok ? 'OK' : 'LOW') : ''}</span></div><div class="big">${roomC ? esc((roomC.detail.match(/[\d.]+ GB/) || ['?'])[0]) : '?'}</div><div class="sub">${roomC ? esc(roomC.detail.replace(/^[\d.]+ GB free -- /, '')) : ''}</div></div>
  </div>
  <div class="card" style="margin-top:16px"><h2>Motors <span class="muted" style="font-size:14px; font-weight:300">temperature</span></h2>
    <div class="motors">${joints.map(j => `<div class="motor">${esc(j.name)} <b>${j.temp} °C</b><div class="gauge"><i style="width:${Math.min(100, j.temp / 90 * 100)}%; ${j.temp > 70 ? 'background:var(--amber)' : ''}"></i></div></div>`).join('')}</div></div>
  <div class="card"><h2>Streams</h2><table class="svc">
    <tr><th>Stream</th><th>State</th><th>Rate</th><th>Detail</th></tr>
    ${(d.streams || []).map(s => `<tr><td>${esc(s.label)}</td><td><span class="${s.ok ? 'g' : 'a'}">${s.ok ? 'arriving' : 'problem'}</span></td><td class="mono">${s.hz} Hz</td><td class="muted" style="font-size:13.5px">${esc(s.note)}</td></tr>`).join('')}
    ${sens && sens.depth ? `<tr><td>Depth camera</td><td><span class="${sens.depth.on ? 'g' : 'a'}">${sens.depth.on ? 'on' : 'off'}</span></td><td class="mono">${sens.depth.fps != null ? Math.round(sens.depth.fps) + ' fps' : '-'}</td><td><span class="muted" style="font-size:13.5px">${esc(sens.depth.note)}</span> <button class="btn ghost" data-act="depth" data-on="${sens.depth.on ? 0 : 1}">${sens.depth.on ? 'Turn off' : 'Turn on'}</button></td></tr>` : ''}
  </table></div>`;
}

/* --------------------------------------------------------------- help */
const HELP = [
  ['The robot page does not load', 'The laptop\'s address is not set, or the robot is still booting (about a minute). Connect page, step 2. Still nothing after two minutes: power-cycle the robot.'],
  ['The clock check is red', 'The robot has no clock battery and wakes up in 1970. Press "Set from this laptop". It is refused while a walk is recording.'],
  ['Odometry stays at zero, or "robot is standing" is red', 'The robot only reports where it is while standing. Stand it up with the remote, then start.'],
  ['The LiDAR counter stays at zero', 'Check the LiDAR is spinning (you can hear it) and its cable. Power-cycle the robot if it stays silent.'],
  ['Make the map says the bag is short', 'The laptop was too busy to keep up. It decodes again at the walk\'s own pace by itself; if it still fails, close heavy programs and try again.'],
  ['Make the map fails straight away', 'Settings, Converting engine: Docker Desktop must be running ("Engine running"). The Developer logs, Job output tab shows the exact error.'],
  ['Foxglove shows an empty 3D panel', 'Open the map file (Open map in Foxglove) rather than the full bag. In Foxglove, import notes\\foxglove\\go2_walk_map.json from the repo once as the layout.'],
  ['The cloud is flat, one layer, no height', 'The LiDAR angle calibration is missing. It is built in for this robot; a different robot needs its own (docs/recording-a-walk.md).'],
  ['Download stops partway', 'The cable came out or the robot rebooted. Press Download again; nothing half-downloaded is kept.'],
  ['Something else', 'Turn on Developer logs (Ctrl+Shift+D), reproduce it, then Export debug bundle and send the zip.'],
];
function renderHelp() {
  const q = ($('#help-search').value || '').toLowerCase();
  $('#help-cards').innerHTML = HELP.filter(h => (h[0] + h[1]).toLowerCase().includes(q)).map(h => `<div class="hcard"><h3>${esc(h[0])}</h3><p>${esc(h[1])}</p></div>`).join('') || '<div class="muted">Nothing matches.</div>';
}
$('#help-search').oninput = renderHelp;

/* ----------------------------------------------------------- settings */
function renderSettings() {
  const s = { ...S.settings, colourmode: S.settings.colour ? 'on' : 'off' }, e = S.engine || {}, pr = S.programs || {};
  const radio = (name, val, label, sub) => `<label><input type="radio" name="${name}" value="${val}" ${s[name] === val ? 'checked' : ''}><div>${label}<small>${sub}</small></div></label>`;
  $('#settings-body').innerHTML = `
    <div class="set"><div><h3>Walks folder</h3><p>Where walks are downloaded and converted. Keep it out of OneDrive.</p></div>
      <div><div class="pathpick" style="margin:0"><input class="input" id="set-walks" value="${esc(s.walks_dir)}"><button class="btn" data-act="pick-walks">${icon('folder')}Choose...</button></div>
      <div class="muted" style="font-size:13px; margin-top:6px">${S.walks_dir.free != null ? fmtB(S.walks_dir.free) + ' free.' : ''} ${S.walks_dir.onedrive ? '<span class="a">Inside OneDrive.</span>' : 'Not synced by OneDrive.'}</div></div></div>
    <div class="set"><div><h3>Map-making engine</h3><p>What turns a recording into the bag and the maps. On most laptops that is Docker Desktop plus the engine itself, a one-time 1 GB download.</p></div>
      <div id="engine-box">${engineHTML()}</div></div>
    <div class="set"><div><h3>Engine choice</h3><p>Auto uses the developer WSL setup if this laptop has it, otherwise Docker Desktop.</p></div>
      <div class="radio">${radio('engine', 'auto', 'Auto', 'now using: ' + esc(e.kind ? e.kind + ' (' + e.detail + ')' : 'nothing: ' + (e.detail || 'checking')))}
        ${radio('engine', 'docker', 'Docker Desktop', 'any Windows laptop; image ' + esc(s.docker_image))}
        ${radio('engine', 'wsl', 'WSL, distro "' + esc(s.wsl_distro) + '"', 'the development laptop only')}</div></div>
    <div class="set"><div><h3>After a walk</h3><p>When a finished walk appears on the robot that is not on this laptop yet.</p></div>
      <div class="radio"><label><input type="radio" name="auto_convert" value="0" ${s.auto_convert ? '' : 'checked'}><div>Ask me <small>the Next step bar offers it</small></div></label>
        <label><input type="radio" name="auto_convert" value="1" ${s.auto_convert ? 'checked' : ''}><div>Download and make the map by itself <small>as soon as the robot is connected and the walk has stopped</small></div></label></div></div>
    <div class="set"><div><h3>Next step bar</h3><p>The bar under the status line that says what to do next. The x on it hides one message.</p></div>
      <div class="radio">${radio('guide', 'all', 'Always', 'status too: connected, ready, not connected')}
        ${radio('guide', 'quiet', 'Only when it needs me', 'recording, downloads, maps, new walks, failures')}
        ${radio('guide', 'off', 'Never', 'the robot pill still shows the connection')}</div></div>
    <div class="set"><div><h3>Camera colour</h3><p>Colours the clean cloud from the camera's colour frames. An experiment: it adds 5-10 minutes a walk and the colours are not reliable yet. Also the check mark beside Make the map.</p></div>
      <div class="radio">${radio('colourmode', 'off', 'Off', 'height colours; the fastest')}
        ${radio('colourmode', 'on', 'On', 'only for walks recorded with the colour stream')}</div></div>
    <div class="set"><div><h3>Keep after converting</h3><p>The full bag is the biggest file by far and is rarely opened.</p></div>
      <div class="radio">${radio('keep', 'all', 'Everything', 'about 5 GB for a 4-minute walk with depth')}
        ${radio('keep', 'maps', 'Maps and point clouds, plus the recording', 'drops the bag; Make the map again rebuilds it')}</div></div>
    <div class="set"><div><h3>Programs</h3><p>Used by the Open buttons. Found automatically.</p></div>
      <dl class="kv"><dt>Foxglove</dt><dd>${pr.foxglove ? '<span class="g">found</span> <span class="mono muted">' + esc(pr.foxglove) + '</span>' : '<span class="a">not found</span> <button class="textlink" data-url="https://foxglove.dev/download">Download page</button>'}</dd>
        <dt>CloudCompare</dt><dd>${pr.cloudcompare ? '<span class="g">found</span> <span class="mono muted">' + esc(pr.cloudcompare) + '</span>' : '<span class="a">not found</span> <button class="textlink" data-url="https://www.cloudcompare.org/release/">Download page</button>'}</dd></dl></div>
    <div class="set"><div><h3>Robot</h3><p>Only change these for a different robot.</p></div>
      <div class="row" style="max-width:420px"><div class="field"><label>Address</label><input class="input mono" id="set-host" value="${esc(s.robot_host)}"></div><div class="field" style="max-width:110px"><label>Port</label><input class="input mono" id="set-port" value="${esc(s.robot_port)}"></div></div></div>
    <div class="set"><div><h3>Developer logs</h3><p>What Studio does behind the scenes: every request, every conversion line, the robot's own logs. Also Ctrl+Shift+D.</p></div>
      <div style="display:flex; gap:10px; align-items:center"><button class="switch" role="switch" aria-checked="${s.dev}" data-act="toggle-dev"><i></i></button><span>Show developer logs</span></div></div>
    <div class="set"><div><h3>About</h3><p>For bug reports.</p></div>
      <dl class="kv"><dt>Studio</dt><dd class="mono">${esc(S.app.version)} (${esc(S.app.commit)})</dd><dt>Logs</dt><dd><button class="textlink" data-act="open-logs">Open logs folder</button> <button class="textlink" data-act="debug-bundle">Export debug bundle</button></dd></dl></div>`;
  $$('#settings-body input[type=radio]').forEach(r => r.onchange = () => saveSettings(r.name === 'colourmode' ? { colour: r.value === 'on' } : { [r.name]: r.name === 'auto_convert' ? r.value === '1' : r.value }));
  $('#set-walks').onchange = e => saveSettings({ walks_dir: e.target.value });
  $('#set-host').onchange = e => saveSettings({ robot_host: e.target.value });
  $('#set-port').onchange = e => saveSettings({ robot_port: +e.target.value });
}
function engineHTML() {
  const e = S.engine || {};
  const job = S.jobs.find(j => j.kind === 'engine');
  const busy = job && (job.state === 'running' || job.state === 'queued');
  const steps = [
    ['Docker Desktop installed', e.kind === 'wsl' || e.docker_installed],
    ['Docker Desktop running', e.kind === 'wsl' || e.docker_running],
    ['Engine downloaded', e.kind === 'wsl' || e.image_present],
  ];
  let h = '';
  if (e.kind === 'wsl') h += `<div class="g" style="font-size:15px">${icon('check', 'g')} Ready: the developer WSL setup ("${esc(e.distro)}") on this laptop.</div>`;
  else h += steps.map(([l, ok], i) => `<div class="report-row">${ok ? icon('check', 'g') : `<span class="mono faint" style="width:18px">${i + 1}</span>`}${l}</div>`).join('');
  if (busy) {
    h += `<div style="margin-top:12px"><div class="stage run"><div class="bar"><i style="width:${job.stage_pct}%"></i></div><div class="lbl"><b>${esc(job.note || 'working')}</b><span>${Math.round(job.stage_pct)}%${job.eta ? ' · ' + fmtEta(job.eta) : ''}</span></div></div>
      <button class="btn danger" style="margin-top:8px" data-act="cancel" data-job="${job.id}">${icon('x')}Cancel</button></div>`;
  } else if (e.step === 'install-docker') {
    h += `<div style="margin-top:12px; font-size:14.5px">Install Docker Desktop, then start it once and accept its terms. It needs administrator rights and may ask to restart Windows.</div>
      <button class="btn primary" style="margin-top:10px" data-url="https://www.docker.com/products/docker-desktop/">${icon('down')}Get Docker Desktop</button>`;
  } else if (e.step === 'start-docker') {
    h += `<div style="margin-top:12px; font-size:14.5px">Docker Desktop is installed but not running. Start it and wait about a minute for "Engine running".</div>
      ${e.docker_exe ? `<button class="btn primary" style="margin-top:10px" data-act="engine-start">${icon('play')}Start Docker Desktop</button>` : ''}`;
  } else if (e.step === 'get-image') {
    h += `<div style="margin-top:12px; font-size:14.5px">One last piece: the engine itself, about 1 GB, downloaded once.</div>
      <div style="display:flex; gap:10px; margin-top:10px; flex-wrap:wrap"><button class="btn primary" data-act="engine-get" data-how="pull">${icon('down')}Download the engine</button>
      <button class="btn" data-act="engine-get" data-how="build">Build it on this laptop instead</button></div>
      <div class="faint" style="font-size:12.5px; margin-top:6px">Build takes about 15 minutes and needs this repo's docker folder; use it if the download is refused.</div>`;
  } else if (e.kind === 'docker') {
    h += `<div class="g" style="margin-top:8px; font-size:15px">${icon('check', 'g')} Ready${e.image_created ? ': engine from ' + esc(e.image_created) : ''}.</div>
      <button class="btn ghost" style="margin-top:6px" data-act="engine-get" data-how="pull">Check for a newer engine</button>`;
  }
  if (job && job.state === 'failed') h += `<div class="err-line" style="margin-top:10px">${icon('warn', 'r')} ${esc(job.error)} <button class="textlink" data-act="joblog" data-job="${job.id}">Log</button></div>`;
  return h + `<div class="devnote" style="margin-top:8px">${esc(JSON.stringify(e))}</div>`;
}
async function saveSettings(ch, quiet) {
  const r = await act('/api/settings', { changes: ch }, quiet ? null : 'Saved.');
  if (r) { S.settings = r; if (page === 'settings') renderSettings(); tickWalks(); }
}

/* -------------------------------------------------------------- actions */
document.addEventListener('click', async e => {
  const cp = e.target.closest('[data-copy]');
  if (cp) { navigator.clipboard.writeText(cp.dataset.copy); cp.textContent = 'Copied'; return; }
  const u = e.target.closest('[data-url]');
  if (u) { act('/api/os', { action: 'open-url', url: u.dataset.url }); return; }
  const t = e.target.closest('[data-act]');
  if (!t) { $$('.menu-list').forEach(m => { m.hidden = true; }); return; }
  const a = t.dataset.act, name = t.dataset.name;
  if (a !== 'menu') $$('.menu-list').forEach(m => { m.hidden = true; });
  switch (a) {
    case 'menu': { const m = t.parentElement.querySelector('.menu-list'); const was = m.hidden; $$('.menu-list').forEach(x => { x.hidden = true; }); m.hidden = !was; break; }
    case 'robotdash': act('/api/os', { action: 'robot-dashboard' }); break;
    case 'setclock': act('/api/robot', { action: 'set-clock' }, 'Robot clock set to this laptop\'s time.'); break;
    case 'addr-on': toast('Windows will ask for permission...'); { const r = await act('/api/os', { action: 'set-address', on: true }); if (r) toast(r.started ? 'Done. The address appears within a few seconds.' : 'Not changed (permission refused?). Use "Do it by hand" instead.', { bad: !r.started }); } break;
    case 'addr-off': { const r = await act('/api/os', { action: 'set-address', on: false }); if (r) toast(r.started ? 'Robot address removed.' : 'Not changed.', { bad: !r.started }); } break;
    case 'stop': stopWalk(); break;
    case 'again': report = null; $('#rec-right').dataset.view = ''; renderRecord(); break;
    case 'marker': if (rec) { const label = 'marker ' + (rec.markers.length + 1); rec.markers.push({ t: (Date.now() - rec.t0) / 1000, label }); saveRec(); toast('Marker dropped: ' + label); } break;
    case 'dl-convert': case 'dl-only': {
      const r = await act('/api/walks', { action: 'download', name, bytes: +(t.dataset.bytes || 0), convert: a === 'dl-convert', colour: !!S.settings.colour }, a === 'dl-convert' ? 'Downloading, then making the map.' : 'Downloading.');
      if (r && page === 'record') go('walks');
      break;
    }
    case 'convert': act('/api/walks', { action: 'convert', name, colour: !!S.settings.colour }, 'Making the map of ' + name + (S.settings.colour ? ', with camera colour.' : '.')); break;
    case 'remap': act('/api/walks', { action: 'map', name, colour: !!S.settings.colour }, 'Rebuilding the map of ' + name + (S.settings.colour ? ', with camera colour.' : '.')); break;
    case 'colour': saveSettings({ colour: t.checked }, true); break;
    case 'colour-menu': saveSettings({ colour: !S.settings.colour }).then(() => toast('Camera colour ' + (S.settings.colour ? 'on: the next map adds 5-10 minutes.' : 'off.'))); break;
    case 'cancel': act('/api/walks', { action: 'cancel', job: t.dataset.job }, 'Cancelling. Nothing half-made is kept.'); break;
    case 'show': act('/api/walks', { action: 'show', name, file: t.dataset.file || '' }); break;
    case 'open': {
      const w = t.dataset.with, prog = S.programs && S.programs[w];
      if (w && !prog) toast((w === 'foxglove' ? 'Foxglove' : 'CloudCompare') + ' is not installed. Settings, Programs has the download page.', { bad: true });
      else act('/api/walks', { action: 'open', name, file: t.dataset.file, with: w });
      break;
    }
    case 'show-walks': act('/api/os', { action: 'show-walks' }); break;
    case 'pick-walks': { const r = await act('/api/os', { action: 'pick-folder', initial: S.settings.walks_dir }); if (r && r.path) saveSettings({ walks_dir: r.path }); break; }
    case 'detail': openDetail(name, t.dataset.to); break;
    case 'saveas': openSaveAs(name); break;
    case 'sa-browse': { const r = await act('/api/os', { action: 'pick-folder', initial: $('#sa-dest').value }); if (r && r.path) $('#sa-dest').value = r.path; break; }
    case 'sa-go': {
      let files = $$('#modal input[data-key]:checked').map(x => x.dataset.key);
      if (files.includes('__raw')) files = files.filter(f => f !== '__raw').concat(t.dataset.raw.split('|').filter(Boolean));
      if (!files.length) { toast('Pick at least one thing to copy.', { bad: true }); break; }
      const r = await act('/api/walks', { action: 'export', name, files, dest: $('#sa-dest').value, zip: $('#sa-zip').checked });
      if (r) { closeOverlays(); watchExport(r.id); }
      break;
    }
    case 'save-notes': act('/api/walks', { action: 'notes', name, notes: { building: $('#d-building').value, floor: $('#d-floor').value, notes: $('#d-notes').value } }, 'Saved with the walk.'); break;
    case 'del-local': confirmBox('Delete ' + name + ' from this laptop?', 'Everything in its folder goes, the recording included. If it is still on the robot you can download it again.', 'Delete', () => act('/api/walks', { action: 'delete-local', name }, 'Deleted from this laptop.').then(tickWalks)); break;
    case 'del-robot': confirmBox('Delete ' + name + ' from the robot?', t.dataset.safe === '1' ? 'A copy is on this laptop.' : 'It is NOT on this laptop. Once deleted it is gone for good.', 'Delete from robot', () => act('/api/robot', { action: 'walk-delete', name }, 'Deleted from the robot.').then(tickWalks)); break;
    case 'close': closeOverlays(); break;
    case 'raw-preflight': showRaw('/api/preflight', R('preflight')); break;
    case 'joblog': setDev(true, true); conJob = t.dataset.job; showTab('job'); break;
    case 'toggle-dev': setDev(!dev()); break;
    case 'open-logs': act('/api/os', { action: 'open-logs' }); break;
    case 'debug-bundle': { const r = await act('/api/os', { action: 'debug-bundle' }); if (r) toast('Saved ' + r.path.split('\\').pop() + ' in Downloads.'); break; }
    case 'engine-get': await act('/api/os', { action: 'engine-get', how: t.dataset.how }, t.dataset.how === 'build' ? 'Building the engine. You can keep using Studio.' : 'Downloading the engine.'); break;
    case 'engine-start': await act('/api/os', { action: 'engine-start-docker' }, 'Starting Docker Desktop. It takes about a minute.'); break;
    case 'depth': { const sens = R('sensors') || {}; if (!DOPT.mode) { DOPT.mode = (sens.wanted || {}).depth_mode || '640x480x15'; DOPT.color = !!(sens.wanted || {}).color; }
      act('/api/robot', { action: 'sensor', params: depthParams(t.dataset.on === '1') }, 'Asked the robot. It takes about 20 s.'); break; }
  }
});

function watchExport(id) {
  toast('Copying...');
  const h = setInterval(() => {
    const j = S.jobs.find(x => x.id === id);
    if (!j || j.state === 'running' || j.state === 'queued') return;
    clearInterval(h);
    if (j.state === 'done') toast('Copy saved.', { actions: [['Show in folder', () => act('/api/walks', { action: 'show', name: j.walk, file: '' })]] });
    else toast('Copy ' + j.state + (j.error ? ': ' + j.error : ''), { bad: true });
  }, 1000);
}

/* ------------------------------------------------------------ overlays */
function openOverlay(sel) { $('#scrim').hidden = false; $(sel).hidden = false; }
function closeOverlays() { $('#scrim').hidden = true; $('#drawer').hidden = true; $('#modal').hidden = true; }
$('#scrim').onclick = closeOverlays;
function confirmBox(title, body, yes, fn) {
  $('#modal').innerHTML = `<h2 style="font-size:21px">${esc(title)}</h2><div class="confirm-body muted">${esc(body)}</div>
    <div style="display:flex; justify-content:flex-end; gap:10px"><button class="btn ghost" data-act="close">Cancel</button><button class="btn primary" id="cf-yes">${esc(yes)}</button></div>`;
  $('#cf-yes').onclick = () => { closeOverlays(); fn(); };
  openOverlay('#modal');
}
function showRaw(title, obj) {
  $('#modal').innerHTML = `<button class="btn ghost x" data-act="close">${icon('x')}</button><h2>${esc(title)}</h2>
    <pre class="mono" style="font-size:12.5px; background:#030504; border:1px solid var(--rule-2); border-radius:8px; padding:12px; overflow:auto; max-height:65vh">${esc(JSON.stringify(obj, null, 2))}</pre>`;
  openOverlay('#modal');
}
let toastH;
function toast(text, o = {}) {
  $('#toast').className = 'toast' + (o.bad ? ' bad' : '');
  $('#toast').querySelector('use').setAttribute('href', o.bad ? '#i-warn' : '#i-check');
  $('#toast-t').textContent = text;
  $('#toast-a').innerHTML = '';
  (o.actions || []).forEach(([label, fn]) => { const b = document.createElement('button'); b.className = 'textlink'; b.textContent = label; b.onclick = fn; $('#toast-a').appendChild(b); });
  $('#toast').hidden = false;
  clearTimeout(toastH);
  toastH = setTimeout(() => { $('#toast').hidden = true; }, o.bad ? 7000 : 4000);
}
document.addEventListener('keydown', e => {
  if (e.key === 'Escape') closeOverlays();
  if (e.ctrlKey && e.shiftKey && e.key.toLowerCase() === 'd') { e.preventDefault(); setDev(!dev()); }
});

/* ------------------------------------------------------ developer logs */
let conTab = 'log', conJob = null, paused = false;
const seen = { log: 0, req: 0, job: 0 };
let logLines = [], reqLines = [], jobLines = [];
async function setDev(on, open) {
  await saveSettings({ dev: on }, true);
  document.body.classList.toggle('dev', on);
  if (on && (open || !document.body.classList.contains('console-open'))) document.body.classList.add('console-open');
}
$('#devtoggle').onclick = () => setDev(!dev());
$('#con-toggle').onclick = () => document.body.classList.toggle('console-open');
$('.con-head').addEventListener('dblclick', () => document.body.classList.toggle('console-open'));
$('#con-pause').onclick = function () { paused = !paused; this.textContent = paused ? 'Resume' : 'Pause'; };
$('#con-copy').onclick = () => navigator.clipboard.writeText($('#con-body').innerText).then(() => toast('Copied the visible log.'));
$('#con-src').onchange = $('#con-lvl').onchange = () => paintCon(true);
function showTab(t) {
  conTab = t;
  $$('.con-tab').forEach(x => x.setAttribute('aria-selected', x.dataset.tab === t));
  paintCon(true);
  if (t === 'sys') loadSystem();
  if (t === 'robot') loadRobotLog();
}
$$('.con-tab').forEach(x => x.onclick = () => { document.body.classList.add('console-open'); showTab(x.dataset.tab); });
const tm = t => new Date(t * 1000).toTimeString().slice(0, 8) + '.' + String(Math.floor(t * 1000) % 1000).padStart(3, '0');
async function pollCon() {
  if (!dev() || paused || !document.body.classList.contains('console-open')) return;
  try {
    if (conTab === 'log') {
      const n = await api.get('/api/logs?since=' + seen.log);
      if (n.length) { seen.log = n[n.length - 1].seq; logLines = logLines.concat(n).slice(-3000); paintCon(); }
    } else if (conTab === 'req') {
      const n = await api.get('/api/logs?kind=req&since=' + seen.req);
      if (n.length) { seen.req = n[n.length - 1].seq; reqLines = reqLines.concat(n).slice(-1500); paintCon(); }
    } else if (conTab === 'job') {
      const job = conJob || (S && S.jobs.length ? (S.jobs.find(j => j.state === 'running') || S.jobs[0]).id : null);
      if (!job) return;
      if (job !== conJob) { conJob = job; jobLines = []; seen.job = 0; }
      const r = await api.get('/api/logs?kind=job&job=' + job + '&since=' + seen.job);
      if (r.lines.length) { seen.job = r.lines[r.lines.length - 1][0]; jobLines = jobLines.concat(r.lines).slice(-4000); }
      jobHead = r.job; paintCon();
    }
  } catch (e) { /* next time */ }
}
let jobHead = null;
function paintCon(reset) {
  const b = $('#con-body');
  const atBottom = b.scrollTop + b.clientHeight >= b.scrollHeight - 30;
  const src = $('#con-src').value, lvl = $('#con-lvl').value;
  $('#con-src').style.display = $('#con-lvl').style.display = conTab === 'log' ? '' : 'none';
  if (conTab === 'log') {
    const keep = logLines.filter(l => (!src || l.src === src) && (!lvl || (lvl === 'err' ? l.level === 'err' : l.level === 'err' || l.level === 'wrn')));
    b.innerHTML = keep.map(l => `<div class="ln"><span class="t">${tm(l.t)}</span>  <span class="${l.level}">${l.level.toUpperCase().padEnd(3)}</span>  <span class="src">${esc(l.src.padEnd(7))}</span><span class="msg">${esc(l.msg)}</span></div>`).join('') || '<div class="faint">Nothing logged yet.</div>';
  } else if (conTab === 'req') {
    b.innerHTML = `<table class="req"><tr class="faint"><td>time</td><td>method</td><td>path</td><td>status</td><td>ms</td><td>bytes</td></tr>${reqLines.map(r => `<tr><td class="t">${tm(r.t)}</td><td>${r.method}</td><td>${esc(r.url)}</td><td class="${r.status === 200 ? 'ok' : 'wrn'}">${esc(r.status || r.error)}</td><td>${r.ms}</td><td>${fmtB(r.bytes)}</td></tr>`).join('')}</table>`;
  } else if (conTab === 'job') {
    const jobs = (S && S.jobs) || [];
    b.innerHTML = `<div style="margin-bottom:6px"><select class="btn" id="con-jobsel" style="padding:2px 6px; font:12px var(--mono)">${jobs.map(j => `<option value="${j.id}" ${j.id === conJob ? 'selected' : ''}>${esc(j.walk)} ${j.kind} (${j.state})</option>`).join('')}</select>
      ${jobHead ? `<span class="faint">${esc(jobHead.engine || '')} · ${esc(jobHead.stage)} ${jobHead.stage_pct}% · ${esc(jobHead.state)}${jobHead.error ? ' · ' + esc(jobHead.error) : ''}</span>` : ''}</div>` +
      (jobLines.map(l => `<div class="ln"><span class="msg">${esc(l[2])}</span></div>`).join('') || '<div class="faint">No output yet. Jobs appear here when you download or make a map.</div>');
    const sel = $('#con-jobsel'); if (sel) sel.onchange = () => { conJob = sel.value; jobLines = []; seen.job = 0; pollCon(); };
  }
  if (reset || atBottom) b.scrollTop = b.scrollHeight;
}
async function loadSystem() {
  try {
    const s = await api.get('/api/system');
    $('#con-body').innerHTML = `<table class="req">${Object.entries(s).map(([k, v]) => `<tr><td class="faint">${esc(k)}</td><td>${esc(typeof v === 'object' ? JSON.stringify(v) : v)}</td></tr>`).join('')}</table>`;
  } catch (e) { $('#con-body').textContent = e.message; }
}
async function loadRobotLog() {
  const names = ['dashboard', 'lidar_relay', 'camera_server', 'realsense_camera'];
  const cur = $('#con-body').dataset.rlog || 'dashboard';
  let body = '';
  try {
    const r = await api.get('/api/robot-logs?name=' + cur);
    body = r.status === 200 && r.body && r.body.error ? `<div class="faint">${esc(r.body.error)}</div>` : r.status === 200 && r.body && r.body.lines ? r.body.lines.map(l => `<div class="ln"><span class="msg">${esc(l)}</span></div>`).join('')
      : r.status === 404 ? '<div class="faint">The robot\'s dashboard is older than this Studio and cannot send its logs yet. Update the robot (Settings in a later version, or deploy_dashboard.sh from the developer laptop).</div>'
        : `<div class="faint">No answer from the robot (${esc(r.error || r.status)}).</div>`;
  } catch (e) { body = esc(e.message); }
  $('#con-body').innerHTML = `<div style="margin-bottom:6px"><select class="btn" id="rlog" style="padding:2px 6px; font:12px var(--mono)">${names.map(n => `<option ${n === cur ? 'selected' : ''}>${n}</option>`).join('')}</select> <button class="btn" id="rlog-refresh" style="padding:2px 8px; font:12px var(--mono)">Refresh</button></div>${body}`;
  $('#con-body').dataset.rlog = cur;
  $('#rlog').onchange = e => { $('#con-body').dataset.rlog = e.target.value; loadRobotLog(); };
  $('#rlog-refresh').onclick = loadRobotLog;
}

/* ------------------------------------------------------------ session
 * Studio is the one app: it follows the robot and the walk from plugging in to
 * an open map, and the Next step bar always offers the one thing to do now.
 * Also here: the robot's clock is set without asking, and (if Settings says
 * so) new walks are downloaded and converted by themselves. */
let clockTried = false;
const dismissed = new Set(JSON.parse(localStorage.getItem('go2.dismissed') || '[]'));
const autoTried = new Set();
const barHidden = new Set();          // messages hidden with the bar's x, until Studio restarts
function dismiss(key) { dismissed.add(key); localStorage.setItem('go2.dismissed', JSON.stringify([...dismissed].slice(-200))); renderGuide(); }
function pendingWalks() {
  const cur = recView() === 'live' || recView() === 'away' ? (rec && rec.name) : null;
  return W.walks.filter(w => w.on_robot && w.stages && !w.stages.downloaded && w.robot_complete !== false && w.name !== cur);
}
function session() {
  if (!S.robot.reachable) return;
  const cc = pfCheck(/clock/);
  if (cc && !cc.ok && !clockTried && recView() !== 'live') {
    clockTried = true;
    api.post('/api/robot', { action: 'set-clock' })
      .then(() => api.post('/api/log', { level: 'ok', msg: 'robot clock was 1970; set from this laptop' }))
      .catch(e => toast('Could not set the robot clock: ' + e.message, true));
  }
  if (S.settings.auto_convert && recView() === 'ready') {
    const busy = S.jobs.some(j => (j.state === 'running' || j.state === 'queued') && j.kind !== 'export');
    const w = pendingWalks().find(x => !autoTried.has(x.name));
    if (w && !busy) {
      autoTried.add(w.name);
      api.post('/api/walks', { action: 'download', name: w.name, bytes: w.bytes || 0, convert: true })
        .then(() => toast(w.name + ' is on the robot: downloading and making the map (Settings, After a walk).'))
        .catch(e => toast(e.message, true));
    }
  }
}
function guideState() {
  const c = S.connect || {}, reach = S.robot.reachable, rv = recView(), pf = R('preflight');
  const st = (R('recordings') || {}).walk || {};
  if (rv === 'live') {
    const w = st.recording || {};
    return { key: 'rec', tone: 'rec', icon: 'rec', text: `Recording <b>${esc(w.name || '')}</b> · <span class="mono">${clock(w.seconds)}</span>`,
      sub: 'Unplug the cable and walk. Plug it back in and press Stop.', actions: [['record', 'Watch it'], ['stop', 'Stop walk', 'danger']] };
  }
  if (rv === 'away') return { key: 'away', tone: 'rec', icon: 'plug', text: `<b>${esc(rec && rec.name)}</b> is recording on the robot, which is out of reach as expected.`,
    sub: 'Plug the cable back in to see it and stop it.', actions: [] };
  const job = S.jobs.find(j => (j.state === 'running' || j.state === 'queued') && j.kind !== 'export' && j.kind !== 'engine');
  if (job) {
    const verb = { download: 'Downloading', convert: 'Making the map of', map: 'Rebuilding the map of' }[job.kind] || 'Working on';
    const stage = { download: 'downloading', unpack: 'unpacking', bag: 'bag', map: 'map', dense: 'clean cloud' }[job.stage] || job.stage;
    return { key: 'job:' + job.id, tone: 'work', icon: 'refresh', text: `${verb} <b>${esc(job.walk)}</b> · ${esc(stage)} ${Math.round(job.stage_pct)}%`,
      sub: (job.eta ? fmtEta(job.eta) + '. ' : '') + 'You can keep using Studio, or record again.', bar: job.pct, actions: [['walks', 'Watch it']] };
  }
  const fail = S.jobs.find(j => j.state === 'failed' && j.kind !== 'export' && !dismissed.has('fail:' + j.id) && Date.now() / 1000 - (j.finished || 0) < 3600);
  if (fail) return { key: 'fail:' + fail.id, tone: 'bad', icon: 'warn', text: `${fail.kind === 'download' ? 'Downloading' : 'Making the map of'} <b>${esc(fail.walk)}</b> failed.`,
    sub: esc(fail.error || ''), actions: [['retry:' + fail.walk + ':' + fail.kind, 'Try again'], ['log:' + fail.id, 'Log'], ['dismiss:fail:' + fail.id, 'Dismiss', 'ghost']] };
  if (reach) {
    const p = pendingWalks().filter(w => !dismissed.has('walk:' + w.name));
    if (p.length) return { key: 'walk:' + p[0].name, tone: 'next', icon: 'down', text: `<b>${esc(p[0].name)}</b> is on the robot but not on this laptop yet${p.length > 1 ? ` (and ${p.length - 1} more)` : ''}.`,
      sub: `${fmtDur(p[0].seconds)}, ${fmtB(p[0].bytes)}. Download it and Studio makes the bag, the map and the clean cloud.`,
      actions: [['dl:' + p[0].name, 'Download and make the map', 'primary'], ['dismiss:walk:' + p[0].name, 'Later', 'ghost']] };
  }
  const done = S.jobs.find(j => j.state === 'done' && (j.kind === 'convert' || j.kind === 'map') && !dismissed.has('done:' + j.id) && Date.now() / 1000 - (j.finished || 0) < 1800);
  if (done) {
    const n = done.walk, w = W.walks.find(x => x.name === n) || {}, st = w.stages || {};
    const made = ['the map', 'the clean cloud', st.colour && 'the colour cloud', (w.plans || []).length && 'floor plans', st.mesh && 'the mesh'].filter(Boolean);
    return { key: 'done:' + done.id, tone: 'done', icon: 'check', text: `<b>${esc(n)}</b> is ready.`, sub: cap(made.slice(0, -1).join(', ') + ' and ' + made[made.length - 1]) + '.',
      actions: [['open:' + n + ':map', 'Map in Foxglove', 'primary'], ['open:' + n + ':' + (st.colour ? 'colour' : 'dense'), st.colour ? 'Colour cloud' : 'Clean cloud'],
        ...((w.plans || []).length ? [['plans:' + n, 'Floor plan']] : []), ['show:' + n, 'Show in folder'], ['dismiss:done:' + done.id, 'Dismiss', 'ghost']] };
  }
  if (!reach) {
    if (!c.link || !c.link.ok) return { key: 'nolink', quiet: true, tone: 'idle', icon: 'plug', text: 'Robot not connected.', sub: 'Turn the robot on and plug the ethernet cable into its dock and into this laptop.', actions: [['connect', 'Connection guide']] };
    if (!c.address || !c.address.ok) return { key: 'noaddr', tone: 'idle', icon: 'shield', text: 'Cable in, but this laptop is not on the robot\'s network yet.', sub: 'It needs the address 192.168.123.99; Studio can add it with one Windows permission prompt.', actions: [['connect', 'Fix it', 'primary']] };
    return { key: 'wait', quiet: true, tone: 'idle', icon: 'refresh', text: 'Waiting for the robot to answer.', sub: 'The robot takes about a minute after power-on. Still nothing after two minutes: power-cycle it.', actions: [['connect', 'Connection guide']] };
  }
  const cc = pfCheck(/clock/);
  if (cc && !cc.ok) return { key: 'clock', quiet: true, tone: 'work', icon: 'refresh', text: 'Setting the robot\'s clock...', sub: 'It wakes up in 1970; a walk recorded like that cannot be lined up.', actions: [['setclock', 'Set it now']] };
  if (pf && !pf.ready) return { key: 'notready', tone: 'idle', icon: 'warn', text: 'Robot connected, not ready to record yet.', sub: 'First: ' + esc(pf.blockers.join(', ')) + '.', actions: [['record', 'See the checks']] };
  return { key: 'ready', quiet: true, tone: 'next', icon: 'check', text: 'Robot connected and ready.', sub: 'Record a walk, or look at the robot live.', actions: [['record', 'Record a walk', 'primary'], ['live', 'Live view']] };
}
function renderGuide() {
  if (!S) return;
  const g = guideState(), el = $('#guide'), mode = S.settings.guide || 'all';
  // The bar can be hidden three ways: Settings, Next step bar, Never; "Only when
  // it needs me", which skips plain status (the robot pill already says it);
  // and the x on a message, which hides that one message until it changes.
  el.hidden = mode === 'off' || (mode === 'quiet' && g.quiet) || barHidden.has(g.key);
  if (el.hidden) return;
  el.className = 'guide ' + g.tone;
  el.innerHTML = `${icon(g.icon, 'gi ' + ({ rec: 'r', work: 'a', bad: 'r', next: 'g', done: 'g' }[g.tone] || 'muted'))}
    <div><div class="gt">${g.text}</div><div class="gs">${g.sub || ''}</div></div>
    <div class="ga">${g.bar != null ? `<span class="gbar"><i style="width:${g.bar}%"></i></span>` : ''}
      ${g.actions.map(([k, label, cls]) => `<button class="btn ${cls || ''}" data-guide="${esc(k)}">${label}</button>`).join('')}
      <button class="gx" data-guide="hide:${esc(g.key)}" title="Hide this message. Settings, Next step bar, hides them all">${icon('x')}</button></div>`;
}
document.addEventListener('click', e => {
  const b = e.target.closest('[data-guide]');
  if (!b) return;
  const k = b.dataset.guide, [a, x, y] = k.split(':');
  if (['record', 'walks', 'live', 'connect'].includes(k)) { go(k); return; }
  if (k === 'stop') { stopWalk(); return; }
  if (k === 'setclock') { act('/api/robot', { action: 'set-clock' }, 'Robot clock set.'); return; }
  if (a === 'dismiss') { dismiss(k.slice(8)); return; }
  if (a === 'hide') { barHidden.add(k.slice(5)); renderGuide(); return; }
  if (a === 'dl') { const w = W.walks.find(w => w.name === x); act('/api/walks', { action: 'download', name: x, bytes: (w && w.bytes) || 0, convert: true }, 'Downloading ' + x + ', then making the map.'); return; }
  if (a === 'retry') { act('/api/walks', y === 'download' ? { action: 'download', name: x, convert: true } : { action: y === 'map' ? 'map' : 'convert', name: x }); return; }
  if (a === 'log') { setDev(true, true); conJob = x; showTab('job'); return; }
  if (a === 'open') { const file = y === 'map' ? `map3d/${x}_map/${x}_map_0.mcap` : `map3d/${x}_${y === 'colour' ? 'colour' : 'dense'}.ply`; act('/api/walks', { action: 'open', name: x, file, with: y === 'map' ? 'foxglove' : 'cloudcompare' }); return; }
  if (a === 'plans') { openDetail(x, 'plans'); return; }
  if (a === 'show') { act('/api/walks', { action: 'show', name: x, file: '' }); return; }
});

/* ------------------------------------------------------------ live: the robot's dashboard, inside Studio */
function renderLive() {
  const frame = $('#live-frame'), off = $('#live-off');
  const url = S.robot.base + '/dashboard-v2?embed=studio';
  if (S.robot.reachable) {
    if (frame.dataset.src !== url) { frame.src = url; frame.dataset.src = url; }
    frame.hidden = false; off.hidden = true;
  } else {
    frame.hidden = true; off.hidden = false;
    off.innerHTML = `<b>The live view appears when the robot is connected</b>It is the robot's own dashboard (3D model, sensors, actions, logs), shown inside Studio. <button class="textlink" data-go="connect">Connection guide</button>`;
  }
}

/* ------------------------------------------------------------------ go */
go(page);
tick(); tickWalks();
setInterval(tick, 1000);
setInterval(tickWalks, 2500);
setInterval(pollCon, 1000);
