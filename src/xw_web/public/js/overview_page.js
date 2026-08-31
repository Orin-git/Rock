/**
 * Overview (merged mission board) — fitted battery + safety cross + compact charts.
 */
import { onState, onObstacle, onMeta, onTask, offState, offObstacle, offMeta, offTask, fetchSensorHub } from '/js/api.js';

const MAX_PTS = 48;
const powerHist = [];
const MAX_LOG = 40;

const C_TEAL = '#0c7f96';
const C_VIOLET = '#a855c8';
const C_GRID = 'rgba(12, 127, 150, 0.12)';
const C_MUTED = 'rgba(91, 108, 128, 0.75)';

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
  const logEl = document.getElementById('dashLog');
  if (!logEl || !text) return;
  const row = document.createElement('div');
  row.className = 'line ' + (level || '');
  const ts = new Date().toLocaleTimeString('zh-CN', { hour12: false });
  row.innerHTML = `<span class="ts">${ts}</span>${escapeHtml(text)}`;
  logEl.prepend(row);
  while (logEl.childNodes.length > MAX_LOG) logEl.removeChild(logEl.lastChild);
}

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
      current: Number(power?.charging_current ?? 0) || 0,
    };
  }
  const t = Date.now() / 1000;
  const pct = 78 + Math.sin(t / 38) * 2.4 + Math.sin(t / 11) * 0.6;
  const volt = 24.55 + (pct - 78) * 0.035 + Math.sin(t / 17) * 0.04;
  return {
    pct: Math.max(55, Math.min(92, pct)),
    volt: Math.max(22.5, Math.min(26.5, volt)),
    simulated: true,
    charging: false,
    docked: false,
    current: 0,
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
      volt: Math.max(22.5, Math.min(26.5, volt)),
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

/** Single-series chart with clear axis */
function drawSeriesChart(canvas, series, opts) {
  const sized = resizeCanvas(canvas);
  if (!sized) return;
  const { ctx, w, h } = sized;
  const padL = 32;
  const padR = 10;
  const padT = 10;
  const padB = 16;
  ctx.clearRect(0, 0, w, h);

  const plotW = w - padL - padR;
  const plotH = h - padT - padB;
  const minY = opts.min;
  const maxY = opts.max;
  const color = opts.color;
  const key = opts.key;
  const unit = opts.unit || '';

  ctx.strokeStyle = C_GRID;
  ctx.lineWidth = 1;
  ctx.font = '10px IBM Plex Mono, monospace';
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  for (let i = 0; i <= 4; i++) {
    const y = padT + (plotH * i) / 4;
    ctx.beginPath();
    ctx.moveTo(padL, y);
    ctx.lineTo(padL + plotW, y);
    ctx.stroke();
    const val = maxY - ((maxY - minY) * i) / 4;
    ctx.fillStyle = color;
    ctx.fillText(opts.intAxis ? String(Math.round(val)) : val.toFixed(1), padL - 5, y);
  }
  ctx.fillStyle = C_MUTED;
  ctx.textAlign = 'center';
  ctx.font = '10px Noto Sans SC, sans-serif';
  ctx.fillText('时间 →', padL + plotW / 2, h - 4);

  if (!series.length) {
    ctx.fillStyle = C_MUTED;
    ctx.font = '12px IBM Plex Mono, monospace';
    ctx.textAlign = 'left';
    ctx.fillText('等待…', padL + 4, padT + plotH / 2);
    return;
  }

  const xAt = (i) => padL + (plotW * i) / Math.max(1, series.length - 1);
  const yAt = (v) => {
    const t = (v - minY) / (maxY - minY || 1);
    return padT + plotH * (1 - Math.max(0, Math.min(1, t)));
  };

  ctx.beginPath();
  series.forEach((p, i) => {
    const x = xAt(i);
    const y = yAt(Number(p[key]));
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.strokeStyle = color;
  ctx.lineWidth = 2.4;
  ctx.stroke();
  if (series.length > 1) {
    ctx.lineTo(xAt(series.length - 1), padT + plotH);
    ctx.lineTo(xAt(0), padT + plotH);
    ctx.closePath();
    ctx.fillStyle = opts.fill || 'rgba(12, 127, 150, 0.12)';
    ctx.fill();
  }

  if (lastPower.simulated && opts.showSim) {
    ctx.fillStyle = '#8b3fad';
    ctx.font = '10px IBM Plex Mono, monospace';
    ctx.textAlign = 'right';
    ctx.fillText('SIM', padL + plotW, padT + 8);
  }

  // latest value badge
  const last = series[series.length - 1];
  if (last) {
    const v = Number(last[key]);
    ctx.fillStyle = color;
    ctx.font = '600 11px IBM Plex Mono, monospace';
    ctx.textAlign = 'left';
    ctx.fillText(`${opts.intAxis ? v.toFixed(0) : v.toFixed(2)}${unit}`, padL + 4, padT + 10);
  }
}

function sourceLabel(src) {
  const s = String(src || '').toLowerCase();
  if (s === 'lidar' || s === 'laser') return '激光';
  if (s === 'depth') return '深度';
  if (s === 'ultra' || s === 'ultrasonic') return '超声';
  return src || '';
}

function drawRadar(canvas, sectors) {
  const parent = canvas.parentElement;
  if (!parent) return;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const size = Math.min(parent.clientWidth || 240, 260);
  canvas.width = size * dpr;
  canvas.height = size * dpr;
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const cx = size / 2;
  const cy = size / 2;
  const R = size * 0.26;
  ctx.clearRect(0, 0, size, size);

  for (let i = 1; i <= 3; i++) {
    ctx.beginPath();
    ctx.arc(cx, cy, (R * i) / 3, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(12, 127, 150, 0.2)';
    ctx.lineWidth = 1;
    ctx.stroke();
  }
  ctx.beginPath();
  ctx.moveTo(cx, cy - R);
  ctx.lineTo(cx, cy + R);
  ctx.moveTo(cx - R, cy);
  ctx.lineTo(cx + R, cy);
  ctx.strokeStyle = 'rgba(12, 127, 150, 0.16)';
  ctx.stroke();

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
    const srcZh = sourceLabel(sec.source);
    const dist = range == null ? 0.55 : Math.max(0.15, Math.min(1, range / 3));
    const r = R * dist;
    const x = cx + Math.cos(d.ang) * r;
    const y = cy + Math.sin(d.ang) * r;

    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, R * 0.95, d.ang - 0.38, d.ang + 0.38);
    ctx.closePath();
    ctx.fillStyle = blocked
      ? 'rgba(194, 49, 69, 0.18)'
      : range != null
        ? 'rgba(15, 138, 90, 0.14)'
        : 'rgba(132, 148, 167, 0.1)';
    ctx.fill();

    ctx.beginPath();
    ctx.arc(x, y, blocked ? 7 : 5.5, 0, Math.PI * 2);
    ctx.fillStyle = blocked ? '#c23145' : range != null ? '#0f8a5a' : '#8494a7';
    ctx.fill();

    const lx = cx + Math.cos(d.ang) * (R + 34);
    const ly = cy + Math.sin(d.ang) * (R + 34);
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.font = '700 12px Noto Sans SC, sans-serif';
    ctx.fillStyle = '#243044';
    ctx.fillText(d.label, lx, ly - 10);
    if (range != null && Number.isFinite(range)) {
      ctx.font = '600 11px IBM Plex Mono, monospace';
      ctx.fillStyle = blocked ? '#c23145' : '#0f8a5a';
      ctx.fillText(`${range.toFixed(2)}m`, lx, ly + 4);
      if (srcZh) {
        ctx.font = '10px Noto Sans SC, sans-serif';
        ctx.fillStyle = '#5b6c80';
        ctx.fillText(srcZh, lx, ly + 16);
      }
    } else {
      ctx.font = '10px IBM Plex Mono, monospace';
      ctx.fillStyle = '#8494a7';
      ctx.fillText('无数据', lx, ly + 6);
    }
  });

  ctx.beginPath();
  ctx.arc(cx, cy, 10, 0, Math.PI * 2);
  ctx.fillStyle = '#fff';
  ctx.fill();
  ctx.strokeStyle = C_TEAL;
  ctx.lineWidth = 2;
  ctx.stroke();
  ctx.fillStyle = C_TEAL;
  ctx.font = '700 9px IBM Plex Mono, monospace';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText('BOT', cx, cy);
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
  const cur = pw.charging && pw.current > 0.01 ? ` · ${pw.current.toFixed(2)}A` : '';
  document.getElementById('dPowerSub').textContent = `${pw.volt.toFixed(1)}V · ${chg}${cur}`;
  tone(document.getElementById('kpiPower'), pw.pct >= 40 ? 'is-ok' : pw.pct >= 20 ? 'is-warn' : 'is-bad');

  const nowPct = document.getElementById('chartNowPct');
  if (nowPct) nowPct.textContent = `当前 ${pw.pct.toFixed(0)}%`;
  const nowVolt = document.getElementById('chartNowVolt');
  if (nowVolt) nowVolt.textContent = `当前 ${pw.volt.toFixed(1)}V`;

  if (pushHist) {
    powerHist.push({
      t: Date.now(),
      pct: pw.pct,
      volt: pw.volt,
    });
    if (powerHist.length > MAX_PTS) powerHist.shift();
  }
}

let lastObstacle = null;

function renderCharts() {
  applyPowerUi(resolvePower(lastState?.power), { pushHist: true });

  const powerCanvas = document.getElementById('chartPower');
  if (powerCanvas) {
    drawSeriesChart(powerCanvas, powerHist, {
      key: 'pct',
      min: 0,
      max: 100,
      color: C_TEAL,
      fill: 'rgba(12, 127, 150, 0.12)',
      unit: '%',
      intAxis: true,
      showSim: true,
    });
  }
  const voltCanvas = document.getElementById('chartVolt');
  if (voltCanvas) {
    drawSeriesChart(voltCanvas, powerHist, {
      key: 'volt',
      min: 22,
      max: 27,
      color: C_VIOLET,
      fill: 'rgba(168, 85, 200, 0.1)',
      unit: 'V',
      intAxis: false,
      showSim: true,
    });
  }

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

let clockTimer = null;
let chartTimer = null;
let sensorTimer = null;
let resizeHandler = null;
let metaHandler = null;
let stateHandler = null;
let obstacleHandler = null;
let taskHandler = null;

export function mount() {
  metaHandler = (m) => {
    const el = document.getElementById('dashIdentity');
    if (el) el.textContent = `XIAOWEI · ${m.robot_id || 'GEN-2'} · DOMAIN ${m.ros_domain_id || '—'}`;
    const up = m.services?.stack_up;
    const dStack = document.getElementById('dStack');
    const dStackSub = document.getElementById('dStackSub');
    if (dStack) dStack.textContent = up ? 'ONLINE' : 'DOWN';
    if (dStackSub) {
      dStackSub.textContent = `${m.services?.supervisor_up ? 'sup:ok' : 'sup:--'} / ${m.services?.map_manager_up ? 'map:ok' : 'map:--'}`;
    }
    tone(document.getElementById('kpiStack'), up ? 'is-ok' : 'is-bad');
  };
  stateHandler = (s) => {
    lastState = s;
    document.getElementById('dMode').textContent = s.mode_name || s.mode || '—';
    document.getElementById('dModeSub').textContent = s.detail || '—';
    tone(document.getElementById('kpiMode'), 'is-ok');
    const isDev = (s.run_mode == null ? 1 : Number(s.run_mode)) !== 0;
    document.getElementById('dRun').textContent = isDev ? '开发者' : '量产';
    tone(document.getElementById('kpiRun'), isDev ? 'is-ok' : 'is-warn');
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
  };
  obstacleHandler = (o) => {
    lastObstacle = o;
    const ok = o.safety_ok !== false && !o.blocked;
    document.getElementById('dSafety').textContent = ok ? 'CLEAR' : 'HOLD';
    document.getElementById('dSafetySub').textContent = o.reason || '—';
    tone(document.getElementById('kpiSafety'), ok ? 'is-ok' : 'is-bad');
    const overall = document.getElementById('safetyOverall');
    if (overall) {
      overall.textContent = ok ? 'CLEAR' : 'HOLD';
      overall.className = ok ? 'pill on' : 'pill off';
    }
    const reason = o.reason || '—';
    const any = o.any_sector_blocked ? ' · 扇区告警' : '';
    const reasonEl = document.getElementById('safetyReason');
    if (reasonEl) reasonEl.textContent = `${reason}${any}`;
    const sec = o.sectors || {};
    const radar = document.getElementById('radarSafety');
    if (radar) drawRadar(radar, sec);
  };
  taskHandler = (line) => {
    const level = /(fail|error|失败|错误|异常)/i.test(line) ? 'err' : /(ok|success|成功|完成)/i.test(line) ? 'ok' : '';
    pushLog(line, level);
  };

  onMeta(metaHandler);
  onState(stateHandler);
  onObstacle(obstacleHandler);
  onTask(taskHandler);

  seedHistory();
  updateClock();
  clockTimer = setInterval(updateClock, 1000);
  chartTimer = setInterval(renderCharts, 1000);
  sensorTimer = setInterval(refreshSensors, 5000);
  refreshSensors();
  renderCharts();
  resizeHandler = () => renderCharts();
  window.addEventListener('resize', resizeHandler, { passive: true });
  pushLog('总览就绪 · 电量未接时使用拟合曲线', 'ok');
  return unmount;
}

export function unmount() {
  if (clockTimer) clearInterval(clockTimer);
  if (chartTimer) clearInterval(chartTimer);
  if (sensorTimer) clearInterval(sensorTimer);
  if (resizeHandler) window.removeEventListener('resize', resizeHandler);
  clockTimer = chartTimer = sensorTimer = resizeHandler = null;
  if (metaHandler) offMeta(metaHandler);
  if (stateHandler) offState(stateHandler);
  if (obstacleHandler) offObstacle(obstacleHandler);
  if (taskHandler) offTask(taskHandler);
  metaHandler = stateHandler = obstacleHandler = taskHandler = null;
}
