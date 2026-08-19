/**
 * Mission-control dashboard — compact layout + fitted battery when real power is absent.
 */
import { connect, onState, onObstacle, onMeta, onTask, fetchSensorHub } from '/js/api.js';
import '/js/app.js';

const MAX_PTS = 48;
const powerHist = [];
const logEl = document.getElementById('dashLog');
const MAX_LOG = 40;

/** Soft teal / violet for light UI charts */
const C_TEAL = '#0c7f96';
const C_VIOLET = '#a855c8';
const C_GRID = 'rgba(12, 127, 150, 0.12)';
const C_MUTED = 'rgba(91, 108, 128, 0.75)';

let lastObstacle = null;
let lastState = null;
let lastPower = { pct: 78, volt: 24.6, simulated: true };

function tone(el, t) {
  if (!el) return;
  el.classList.remove('is-ok', 'is-bad', 'is-warn', 'is-neutral');
  el.classList.add(t || 'is-neutral');
}

function escapeHtml(s) {
  return String(s)
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;');
}

function pushLog(text, level) {
  if (!logEl || !text) return;
  const row = document.createElement('div');
  row.className = 'line ' + (level || '');
  const ts = new Date().toLocaleTimeString('zh-CN', { hour12: false });
  row.innerHTML = `<span class="ts">${ts}</span>${escapeHtml(text)}`;
  logEl.prepend(row);
  while (logEl.childNodes.length > MAX_LOG) logEl.removeChild(logEl.lastChild);
}

/**
 * Real battery not wired yet → fitted demo curve.
 * Uses real values only when percent looks alive (> 1%).
 */
function resolvePower(power) {
  const rawPct = Number(power?.battery_percent ?? 0);
  const rawV = Number(power?.voltage ?? 0);
  if (Number.isFinite(rawPct) && rawPct > 1) {
    return {
      pct: Math.max(0, Math.min(100, rawPct)),
      volt: Number.isFinite(rawV) && rawV > 1 ? rawV : 24 + rawPct * 0.04,
      simulated: false,
      charging: !!power?.charging,
      docked: !!power?.docked,
    };
  }
  // Slow breathing drain around ~78%
  const t = Date.now() / 1000;
  const pct = 78 + Math.sin(t / 38) * 2.4 + Math.sin(t / 11) * 0.6;
  const volt = 24.55 + (pct - 78) * 0.035 + Math.sin(t / 17) * 0.04;
  return {
    pct: Math.max(55, Math.min(92, pct)),
    volt: Math.max(22.5, Math.min(26.5, volt)),
    simulated: true,
    charging: false,
    docked: false,
  };
}

function seedHistory() {
  const now = Date.now();
  for (let i = MAX_PTS - 1; i >= 0; i--) {
    const t = now - i * 1000;
    const sec = t / 1000;
    const pct = 78 + Math.sin(sec / 38) * 2.4 + Math.sin(sec / 11) * 0.6;
    const volt = 24.55 + (pct - 78) * 0.035;
    powerHist.push({
      t,
      pct: Math.max(55, Math.min(92, pct)),
      v10: Math.min(100, Math.max(0, volt * 4)),
    });
  }
}

function resizeCanvas(canvas) {
  const parent = canvas.parentElement;
  if (!parent) return null;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const w = parent.clientWidth || 300;
  const h = parent.clientHeight || 140;
  canvas.width = Math.floor(w * dpr);
  canvas.height = Math.floor(h * dpr);
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h };
}

function drawSparkline(canvas, series) {
  const sized = resizeCanvas(canvas);
  if (!sized) return;
  const { ctx, w, h } = sized;
  const pad = 10;
  ctx.clearRect(0, 0, w, h);

  ctx.strokeStyle = C_GRID;
  ctx.lineWidth = 1;
  for (let i = 0; i < 4; i++) {
    const y = pad + ((h - pad * 2) * i) / 3;
    ctx.beginPath();
    ctx.moveTo(pad, y);
    ctx.lineTo(w - pad, y);
    ctx.stroke();
  }

  if (!series.length) {
    ctx.fillStyle = C_MUTED;
    ctx.font = '12px IBM Plex Mono, monospace';
    ctx.fillText('等待遥测…', pad + 4, h / 2);
    return;
  }

  const drawSeries = (key, color, fill) => {
    const vals = series.map((p) => Number(p[key]));
    ctx.beginPath();
    vals.forEach((v, i) => {
      const x = pad + ((w - pad * 2) * i) / Math.max(1, series.length - 1);
      const t = Math.max(0, Math.min(1, v / 100));
      const y = h - pad - t * (h - pad * 2);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.strokeStyle = color;
    ctx.lineWidth = 2.25;
    ctx.stroke();
    if (fill && vals.length > 1) {
      ctx.lineTo(pad + (w - pad * 2), h - pad);
      ctx.lineTo(pad, h - pad);
      ctx.closePath();
      ctx.fillStyle = 'rgba(12, 127, 150, 0.1)';
      ctx.fill();
    }
  };

  drawSeries('pct', C_TEAL, true);
  drawSeries('v10', C_VIOLET, false);

  ctx.font = '11px IBM Plex Mono, monospace';
  ctx.fillStyle = C_TEAL;
  ctx.fillText('BAT %', pad, 14);
  ctx.fillStyle = C_VIOLET;
  ctx.fillText('V×4', pad + 52, 14);
  if (lastPower.simulated) {
    ctx.fillStyle = '#8b3fad';
    ctx.fillText('SIM', w - pad - 28, 14);
  }
}

function drawRadar(canvas, sectors) {
  const parent = canvas.parentElement;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const size = Math.min(parent.clientWidth || 200, parent.clientHeight || 200, 200);
  canvas.width = size * dpr;
  canvas.height = size * dpr;
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const cx = size / 2;
  const cy = size / 2;
  const R = size * 0.36;
  ctx.clearRect(0, 0, size, size);

  for (let i = 1; i <= 3; i++) {
    ctx.beginPath();
    ctx.arc(cx, cy, (R * i) / 3, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(12, 127, 150, 0.18)';
    ctx.lineWidth = 1;
    ctx.stroke();
  }
  ctx.beginPath();
  ctx.moveTo(cx, cy - R);
  ctx.lineTo(cx, cy + R);
  ctx.moveTo(cx - R, cy);
  ctx.lineTo(cx + R, cy);
  ctx.strokeStyle = 'rgba(12, 127, 150, 0.14)';
  ctx.stroke();

  if (document.documentElement.dataset.fx === 'high') {
    const ang = (Date.now() / 1800) % (Math.PI * 2);
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, R, ang - 0.45, ang);
    ctx.closePath();
    ctx.fillStyle = 'rgba(12, 127, 150, 0.1)';
    ctx.fill();
  }

  const dirs = [
    { key: 'front', label: '前', ang: -Math.PI / 2 },
    { key: 'right', label: '右', ang: 0 },
    { key: 'rear', label: '后', ang: Math.PI / 2 },
    { key: 'left', label: '左', ang: Math.PI },
  ];

  dirs.forEach((d) => {
    const sec = (sectors && sectors[d.key]) || {};
    const blocked = !!sec.blocked;
    const range = sec.range_m != null ? Number(sec.range_m) : null;
    const dist = range == null ? 0.55 : Math.max(0.12, Math.min(1, range / 3));
    const r = R * dist;
    const x = cx + Math.cos(d.ang) * r;
    const y = cy + Math.sin(d.ang) * r;

    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, R * 0.92, d.ang - 0.35, d.ang + 0.35);
    ctx.closePath();
    ctx.fillStyle = blocked
      ? 'rgba(194, 49, 69, 0.16)'
      : range != null
        ? 'rgba(15, 138, 90, 0.12)'
        : 'rgba(132, 148, 167, 0.1)';
    ctx.fill();

    ctx.beginPath();
    ctx.arc(x, y, blocked ? 7 : 5.5, 0, Math.PI * 2);
    ctx.fillStyle = blocked ? '#c23145' : range != null ? '#0f8a5a' : '#8494a7';
    ctx.fill();

    const lx = cx + Math.cos(d.ang) * (R + 16);
    const ly = cy + Math.sin(d.ang) * (R + 16);
    ctx.fillStyle = '#5b6c80';
    ctx.font = '600 11px Noto Sans SC, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(d.label, lx, ly);
    if (range != null) {
      ctx.font = '10px IBM Plex Mono, monospace';
      ctx.fillStyle = '#8494a7';
      ctx.fillText(range.toFixed(2) + 'm', lx, ly + 12);
    }
  });

  ctx.beginPath();
  ctx.arc(cx, cy, 9, 0, Math.PI * 2);
  ctx.fillStyle = '#fff';
  ctx.fill();
  ctx.strokeStyle = C_TEAL;
  ctx.lineWidth = 2;
  ctx.stroke();
}

function drawGauge(canvas, value, max, label, color) {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const size = 96;
  canvas.width = size * dpr;
  canvas.height = size * dpr;
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const cx = size / 2;
  const cy = size / 2;
  const r = 34;
  const start = Math.PI * 0.75;
  const end = Math.PI * 2.25;
  const t = Math.max(0, Math.min(1, value / max));

  ctx.clearRect(0, 0, size, size);
  ctx.beginPath();
  ctx.arc(cx, cy, r, start, end);
  ctx.strokeStyle = 'rgba(15, 27, 45, 0.08)';
  ctx.lineWidth = 7;
  ctx.lineCap = 'round';
  ctx.stroke();

  ctx.beginPath();
  ctx.arc(cx, cy, r, start, start + (end - start) * t);
  ctx.strokeStyle = color;
  ctx.stroke();

  ctx.fillStyle = '#0f1b2d';
  ctx.font = '700 16px Space Grotesk, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(label, cx, cy);
}

function updateClock() {
  const el = document.getElementById('dashClock');
  if (!el) return;
  const now = new Date();
  const t = now.toLocaleTimeString('zh-CN', { hour12: false });
  const d = now.toLocaleDateString('zh-CN', { weekday: 'short', month: '2-digit', day: '2-digit' });
  el.innerHTML = `${t}<small>${d} · LOCAL</small>`;
}

function applyPowerUi(pw, { pushHist = true } = {}) {
  lastPower = pw;
  const el = document.getElementById('dPower');
  el.innerHTML = `${pw.pct.toFixed(0)}%${pw.simulated ? '<span class="sim-tag">SIM</span>' : ''}`;
  const chg = pw.charging ? '充电中' : pw.docked ? '已对接' : pw.simulated ? '拟合放电' : '放电';
  document.getElementById('dPowerSub').textContent = `${pw.volt.toFixed(1)}V · ${chg}`;
  tone(document.getElementById('kpiPower'), pw.pct >= 40 ? 'is-ok' : pw.pct >= 20 ? 'is-warn' : 'is-bad');

  if (pushHist) {
    powerHist.push({
      t: Date.now(),
      pct: pw.pct,
      v10: Math.min(100, Math.max(0, pw.volt * 4)),
    });
    if (powerHist.length > MAX_PTS) powerHist.shift();
  }
}

function renderCharts() {
  // Refresh fitted power once per chart tick (1 Hz)
  applyPowerUi(resolvePower(lastState?.power), { pushHist: true });

  const powerCanvas = document.getElementById('chartPower');
  if (powerCanvas) drawSparkline(powerCanvas, powerHist);

  const radar = document.getElementById('radarSafety');
  if (radar) drawRadar(radar, lastObstacle?.sectors);

  const batt = lastPower.pct;
  const locCode = Number(lastState?.localization_status ?? (lastState?.localization_ok ? 0 : 1));
  const locPts = locCode === 0 ? 30 : locCode === 2 ? 15 : 5;
  const health =
    (lastState?.safety_ok !== false ? 35 : 0) +
    (!lastState?.emergency_stop ? 35 : 0) +
    locPts;

  const gBatt = document.getElementById('gaugeBatt');
  const gHealth = document.getElementById('gaugeHealth');
  if (gBatt) {
    drawGauge(
      gBatt,
      batt,
      100,
      `${batt.toFixed(0)}%`,
      batt >= 40 ? '#0f8a5a' : batt >= 20 ? '#a37818' : '#c23145',
    );
  }
  if (gHealth) {
    drawGauge(
      gHealth,
      health,
      100,
      `${health}`,
      health >= 70 ? '#0f8a5a' : health >= 40 ? '#a37818' : '#c23145',
    );
  }
}

async function refreshSensors() {
  const box = document.getElementById('dashSensors');
  if (!box) return;
  const j = await fetchSensorHub();
  const sensors = j.sensors || {};
  const order = ['lidar', 'depth_camera', 'depth_camera_2', 'chassis', 'imu', 'ultrasonic'];
  const labels = {
    lidar: '激光雷达',
    depth_camera: '前上深度',
    depth_camera_2: '前下深度',
    chassis: '底盘',
    imu: 'IMU',
    ultrasonic: '超声波',
  };
  box.innerHTML = '';
  order.forEach((key) => {
    const s = sensors[key] || {};
    const div = document.createElement('div');
    let st = '—';
    let cls = 'is-warn';
    if (s.status === 'live' || s.present) {
      st = 'ONLINE';
      cls = 'is-ok';
    } else if (s.status === 'partial') {
      st = 'PARTIAL';
      cls = 'is-warn';
    } else if (s.status === 'placeholder') {
      st = 'N/A';
      cls = 'is-warn';
    } else if (s.status === 'missing' || s.present === false) {
      st = 'OFF';
      cls = 'is-bad';
    }
    div.className = 'dash-sensor ' + cls;
    div.innerHTML = `<span class="name">${labels[key] || key}</span><span class="st">${st}</span>`;
    box.appendChild(div);
  });
}

onMeta((m) => {
  document.getElementById('dashIdentity').textContent =
    `XIAOWEI · ${m.robot_id || 'GEN-2'} · DOMAIN ${m.ros_domain_id || '—'}`;
  const up = m.services?.stack_up;
  document.getElementById('dStack').textContent = up ? 'ONLINE' : 'DOWN';
  document.getElementById('dStackSub').textContent =
    `${m.services?.supervisor_up ? 'sup:ok' : 'sup:--'} / ${m.services?.map_manager_up ? 'map:ok' : 'map:--'}`;
  tone(document.getElementById('kpiStack'), up ? 'is-ok' : 'is-bad');
});

onState((s) => {
  lastState = s;
  document.getElementById('dMode').textContent = s.mode_name || s.mode || '—';
  document.getElementById('dModeSub').textContent = s.detail || '—';
  tone(document.getElementById('kpiMode'), 'is-ok');

  applyPowerUi(resolvePower(s.power), { pushHist: false });

  const estop = !!s.emergency_stop;
  document.getElementById('dEstop').textContent = estop ? 'DISABLED' : '使能';
  document.getElementById('dProfile').textContent = `profile ${s.profile || '—'}`;
  tone(document.getElementById('kpiEstop'), estop ? 'is-bad' : 'is-ok');

  const locCode = Number(s.localization_status ?? (s.localization_ok ? 0 : 1));
  const locLabels = { 0: '正常', 1: '未就绪', 2: '漂移自愈', 3: '需重定位' };
  const locTones = { 0: 'is-ok', 1: 'is-warn', 2: 'is-warn', 3: 'is-bad' };
  document.getElementById('dLoc').textContent = locLabels[locCode] ?? `LOC ${locCode}`;
  document.getElementById('dMap').textContent = s.active_map ? `map ${s.active_map}` : 'map —';
  tone(document.getElementById('kpiLoc'), locTones[locCode] || 'is-warn');
});

onObstacle((o) => {
  lastObstacle = o;
  const ok = o.safety_ok !== false && !o.blocked;
  document.getElementById('dSafety').textContent = ok ? 'CLEAR' : 'HOLD';
  document.getElementById('dSafetySub').textContent = o.reason || '—';
  tone(document.getElementById('kpiSafety'), ok ? 'is-ok' : 'is-bad');
});

onTask((line) => {
  const level = /(fail|error|失败|错误|异常)/i.test(line)
    ? 'err'
    : /(ok|success|成功|完成)/i.test(line)
      ? 'ok'
      : '';
  pushLog(line, level);
});

seedHistory();
connect();
if (window.XwHudFx) {
  window.XwHudFx.startParticles(document.body);
}

updateClock();
setInterval(updateClock, 1000);
setInterval(renderCharts, 1000);
setInterval(refreshSensors, 5000);
refreshSensors();
renderCharts();
window.addEventListener('resize', () => renderCharts(), { passive: true });

pushLog('态势大屏就绪 · 电量未接时使用拟合曲线', 'ok');
