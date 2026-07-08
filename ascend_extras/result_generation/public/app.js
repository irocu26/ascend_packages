/* ASCEND result dashboard — live client.
 * Streams mission state via Server-Sent Events and renders the arena map,
 * task checklist, telemetry and timeline. Polling fallback if SSE drops. */
'use strict';

const SVG_NS = 'http://www.w3.org/2000/svg';
const STATUS_GLYPH = { done: '✔', active: '⟳', failed: '✖', pending: '○' };

const el = (id) => document.getElementById(id);
const fmtElapsed = (s) => {
  s = Math.max(0, s | 0);
  const m = Math.floor(s / 60);
  return String(m).padStart(2, '0') + ':' + String(s % 60).padStart(2, '0');
};
const sgn = (v) => (v >= 0 ? '+' : '') + v.toFixed(2);

let lastData = null;

/* ── Connection handling ──────────────────────────────── */
function setConn(state) {
  const c = el('conn');
  c.className = 'conn ' + state;
  el('conn-text').textContent =
    state === 'live' ? 'live' : state === 'dead' ? 'disconnected' : 'connecting…';
}

function connect() {
  setConn('');
  let es;
  try {
    es = new EventSource('/api/stream');
  } catch (e) {
    return startPolling();
  }
  es.onmessage = (ev) => {
    setConn('live');
    try { render(JSON.parse(ev.data)); } catch (e) { /* ignore */ }
  };
  es.onerror = () => {
    setConn('dead');
    es.close();
    setTimeout(connect, 2000); // auto-reconnect
  };
}

let pollTimer = null;
function startPolling() {
  if (pollTimer) return;
  pollTimer = setInterval(async () => {
    try {
      const r = await fetch('/api/state');
      render(await r.json());
      setConn('live');
    } catch (e) { setConn('dead'); }
  }, 1000);
}

/* ── Clock ────────────────────────────────────────────── */
setInterval(() => {
  el('clock').textContent = new Date().toLocaleTimeString();
}, 1000);

/* ── Main render ──────────────────────────────────────── */
function render(d) {
  lastData = d;
  const m = d.mission, t = d.telemetry;

  el('state-value').textContent = m.state;
  el('state-sub').textContent = m.state_label;
  const stateCard = el('state-card');
  stateCard.style.borderLeftColor = m.in_failsafe ? 'var(--red)'
    : m.complete ? 'var(--green)' : 'var(--accent)';
  el('state-value').style.color = m.in_failsafe ? 'var(--red)'
    : m.complete ? 'var(--green)' : 'var(--accent)';

  el('elapsed').textContent = fmtElapsed(m.elapsed_s);
  el('tasks-count').textContent = `${m.tasks_done} / ${m.tasks_total}`;
  el('features-count').textContent = `${d.features.found} / ${d.features.total}`;

  // Battery
  const bat = t.battery_seen ? t.battery : null;
  el('battery').textContent = bat === null ? '—' : bat.toFixed(0) + '%';
  const bar = el('battery-bar');
  bar.style.width = (bat === null ? 0 : Math.max(0, Math.min(100, bat))) + '%';
  bar.style.background = bat === null ? 'var(--grey)'
    : bat < 20 ? 'var(--red)' : bat < 40 ? 'var(--yellow)' : 'var(--green)';

  // Telemetry row
  el('pos-x').textContent = sgn(t.x);
  el('pos-y').textContent = sgn(t.y);
  el('pos-z').textContent = t.z.toFixed(2);
  el('pos-src').textContent = t.pose_source;
  el('survey-wp').textContent = d.survey.progress +
    (d.survey.total_wp ? ` (${d.survey.pct.toFixed(0)}%)` : '');

  el('arena-dims').textContent = `${d.arena.x.toFixed(2)} × ${d.arena.y.toFixed(2)} m`;

  // Failsafe banner
  const fb = el('failsafe-banner');
  if (m.failsafe) {
    fb.textContent = '⚠ FAILSAFE — ' + m.failsafe;
    fb.classList.remove('hidden');
  } else {
    fb.classList.add('hidden');
  }

  renderTasks(d.tasks);
  renderFeatures(d.features.list);
  renderLog(d.events);
  drawArena(d);

  // If a report was generated server-side, surface the links.
  if (d.report_paths && Object.keys(d.report_paths).length) {
    showReportLinks(Object.keys(d.report_paths));
  }
}

function renderTasks(tasks) {
  const ul = el('tasks');
  ul.innerHTML = '';
  for (const task of tasks) {
    const li = document.createElement('li');
    li.className = task.status;
    const detail = task.detail
      ? task.detail
      : (task.duration_s != null ? task.duration_s.toFixed(0) + 's' : '');
    li.innerHTML =
      `<span class="glyph">${STATUS_GLYPH[task.status]}</span>` +
      `<span class="t-label">${esc(task.label)}</span>` +
      `<span class="t-detail">${esc(detail)}</span>`;
    ul.appendChild(li);
  }
}

function renderFeatures(list) {
  const body = el('features-body');
  if (!list.length) {
    body.innerHTML = '<tr class="empty"><td colspan="4">No features located yet.</td></tr>';
    return;
  }
  body.innerHTML = list.map((f) =>
    `<tr><td>F${esc(f.seed_id)}</td><td>${sgn(f.x)}</td>` +
    `<td>${sgn(f.y)}</td><td>${f.confidence.toFixed(2)}</td></tr>`).join('');
}

function renderLog(events) {
  const ul = el('log');
  ul.innerHTML = events.map((e) => {
    const ts = (e.t_iso || '').slice(11, 19);
    return `<li class="kind-${esc(e.kind)}"><span class="ts">${ts}</span>` +
      `<span>${esc(e.text)}</span></li>`;
  }).join('');
}

/* ── Arena map (SVG) ──────────────────────────────────── */
function drawArena(d) {
  const svg = el('arena');
  const a = d.arena;
  const pad = 1.0;
  const vw = a.x + 2 * pad, vh = a.y + 2 * pad;
  const W = 700, H = Math.round(W * vh / vw);
  svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
  const sc = W / vw;
  const X = (mx) => (mx + pad) * sc;
  const Y = (my) => (vh - (my + pad)) * sc; // flip y up

  let s = `<rect x="0" y="0" width="${W}" height="${H}" fill="#0d1320"/>`;

  // grid
  for (let gx = 0; gx <= a.x + 1e-6; gx++)
    s += line(X(gx), Y(0), X(gx), Y(a.y), '#1d2740', 1);
  for (let gy = 0; gy <= a.y + 1e-6; gy++)
    s += line(X(0), Y(gy), X(a.x), Y(gy), '#1d2740', 1);

  // boundary
  s += `<rect x="${X(0)}" y="${Y(a.y)}" width="${a.x * sc}" height="${a.y * sc}" ` +
       `fill="none" stroke="#f4d03f" stroke-width="3"/>`;

  // base zone
  const bz = 1.5;
  s += `<rect x="${X(a.dock_x)}" y="${Y(a.dock_y + bz)}" width="${bz * sc}" ` +
       `height="${bz * sc}" fill="rgba(244,208,63,0.12)" stroke="#f4d03f" ` +
       `stroke-width="1" stroke-dasharray="5 4"/>`;
  s += text(X(a.dock_x) + 5, Y(a.dock_y) - 5, 'BASE', '#f4d03f', 12);

  // trail
  if (d.trail && d.trail.length > 1) {
    const pts = d.trail.map((p) => `${X(p[0]).toFixed(1)},${Y(p[1]).toFixed(1)}`).join(' ');
    s += `<polyline points="${pts}" fill="none" stroke="#38bdf8" ` +
         `stroke-width="2.5" opacity="0.75"/>`;
  }

  // features
  for (const f of d.features.list) {
    const cx = X(f.x), cy = Y(f.y);
    s += star(cx, cy, 11, '#2ecc71');
    s += text(cx + 13, cy + 4, `F${f.seed_id}`, '#2ecc71', 12);
  }

  // drone
  if (d.mission.started) {
    const dx = X(d.telemetry.x), dy = Y(d.telemetry.y);
    s += `<circle cx="${dx}" cy="${dy}" r="14" fill="none" stroke="#e74c3c" ` +
         `stroke-width="1.5" opacity="0.5"><animate attributeName="r" ` +
         `values="9;16;9" dur="1.6s" repeatCount="indefinite"/></circle>`;
    s += `<circle cx="${dx}" cy="${dy}" r="8" fill="#e74c3c" stroke="#fff" stroke-width="2"/>`;
  }

  svg.innerHTML = s;
}

const line = (x1, y1, x2, y2, c, w) =>
  `<line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" stroke="${c}" stroke-width="${w}"/>`;
const text = (x, y, t, c, sz) =>
  `<text x="${x}" y="${y}" fill="${c}" font-size="${sz}" font-family="monospace">${esc(t)}</text>`;
function star(cx, cy, r, color) {
  let pts = [];
  for (let i = 0; i < 10; i++) {
    const ang = -Math.PI / 2 + i * Math.PI / 5;
    const rad = i % 2 === 0 ? r : r * 0.45;
    pts.push(`${(cx + rad * Math.cos(ang)).toFixed(1)},${(cy + rad * Math.sin(ang)).toFixed(1)}`);
  }
  return `<polygon points="${pts.join(' ')}" fill="${color}"/>`;
}

/* ── Report download ──────────────────────────────────── */
el('btn-report').addEventListener('click', async () => {
  const btn = el('btn-report');
  btn.disabled = true;
  btn.textContent = '⏳ Generating report…';
  try {
    const r = await fetch('/api/report', { method: 'POST' });
    const j = await r.json();
    if (j.ok) {
      showReportLinks(Object.keys(j.paths));
      // Auto-open the HTML report in a new tab.
      if (j.paths.html) window.open('/api/report/file/html', '_blank');
    }
  } catch (e) {
    alert('Report generation failed: ' + e);
  } finally {
    btn.disabled = false;
    btn.textContent = '⬇ Generate & Download Report';
  }
});

function showReportLinks(formats) {
  const labels = { html: 'View HTML', pdf: 'Download PDF', json: 'Download JSON' };
  el('report-links').innerHTML = formats
    .map((f) => `<a href="/api/report/file/${f}" target="_blank">${labels[f] || f}</a>`)
    .join('');
}

/* ── util ─────────────────────────────────────────────── */
function esc(v) {
  return String(v).replace(/[&<>"']/g, (c) =>
    ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

connect();
