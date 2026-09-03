/**
 * Overview (merged mission board) — real battery + safety cross + compact charts.
 */
import {
  onState,
  onObstacle,
  onMeta,
  onTask,
  offState,
  offObstacle,
  offMeta,
  offTask,
  fetchSensorHub,
  fetchPowerHistory,
} from '/js/api.js';

/** Calendar-day axis (00:00–24:00 local); points come from server history. */
const DAY_MS = 24 * 60 * 60 * 1000;
const TICK_HOURS = 2; // 12 ticks across the day
const powerHist = [];
let dayStartMs = 0;
let dayEndMs = 0;
let dayLabel = '';
/** Selected calendar day YYYY-MM-DD; null until first history load. */
let selectedDate = null;
let viewingToday = true;
let weekDays = [];
const MAX_LOG = 40;

const C_TEAL = '#0c7f96';
const C_VIOLET = '#a855c8';
const C_GRID = 'rgba(12, 127, 150, 0.12)';
const C_MUTED = 'rgba(91, 108, 128, 0.75)';

let lastState = null;
let lastPower = { pct: 0, volt: 0, simulated: false, available: false };

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
  const available = Number.isFinite(rawPct) && rawPct > 0.05 && Number.isFinite(rawV) && rawV > 1;
  if (available) {
    return {
      pct: Math.max(0, Math.min(100, rawPct)),
      volt: rawV,
      simulated: false,
      available: true,
      charging: !!power?.charging,
      docked: !!power?.docked,
      current: Number(power?.charging_current ?? 0) || 0,
      etaMin: Number(power?.time_to_full_min),
    };
  }
  return {
    pct: Number.isFinite(rawPct) ? Math.max(0, Math.min(100, rawPct)) : 0,
    volt: Number.isFinite(rawV) ? Math.max(0, rawV) : 0,
    simulated: false,
    available: false,
    charging: !!power?.charging,
    docked: !!power?.docked,
    current: Number(power?.charging_current ?? 0) || 0,
    etaMin: Number(power?.time_to_full_min),
  };
}

function seedHistory() {
  powerHist.length = 0;
  const now = new Date();
  const start = new Date(now.getFullYear(), now.getMonth(), now.getDate());
  dayStartMs = start.getTime();
  dayEndMs = dayStartMs + DAY_MS;
  dayLabel = `${String(now.getMonth() + 1).padStart(2, '0')}-${String(now.getDate()).padStart(2, '0')}`;
}

function formatDayTitle(dateStr, isToday) {
  if (isToday) return '今日';
  if (!dateStr) return '—';
  const parts = String(dateStr).split('-');
  if (parts.length === 3) return `${parts[1]}-${parts[2]}`;
  return dateStr;
}

function updateChartDayLabels() {
  const title = formatDayTitle(selectedDate || dayLabel, viewingToday);
  const pctEl = document.getElementById('chartDayPctLabel');
  const voltEl = document.getElementById('chartDayVoltLabel');
  if (pctEl) pctEl.textContent = title;
  if (voltEl) voltEl.textContent = title;
  const live = document.getElementById('chartLiveDot');
  if (live) {
    live.style.visibility = viewingToday ? 'visible' : 'hidden';
    live.title = viewingToday ? '今日实时采样 · 服务端落盘' : `${title} 历史曲线`;
  }
}

function renderWeekBar() {
  const bar = document.getElementById('powerWeekBar');
  if (!bar) return;
  if (!weekDays.length) {
    bar.innerHTML = '';
    return;
  }
  const sel = selectedDate || weekDays.find((d) => d.is_today)?.date || '';
  bar.innerHTML = weekDays
    .map((d) => {
      const cls = [
        'power-day-chip',
        d.date === sel ? 'is-active' : '',
        d.is_today ? 'is-today' : '',
        d.is_future ? 'is-future' : '',
        d.has_data ? 'has-data' : 'no-data',
      ]
        .filter(Boolean)
        .join(' ');
      const disabled = d.is_future ? 'disabled aria-disabled="true"' : '';
      const mark = d.has_data ? '<i class="dot"></i>' : '';
      return `<button type="button" class="${cls}" data-date="${escapeHtml(d.date)}" role="tab" aria-selected="${d.date === sel}" ${disabled}><span class="wd">${escapeHtml(d.weekday)}</span><span class="dn">${d.day}</span>${mark}</button>`;
    })
    .join('');
}

function onWeekBarClick(ev) {
  const btn = ev.target.closest('button.power-day-chip');
  if (!btn || btn.disabled) return;
  const date = btn.getAttribute('data-date');
  if (!date || date === selectedDate) return;
  selectedDate = date;
  loadPowerHistory().then(() => renderCharts());
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

async function loadPowerHistory() {
  const j = await fetchPowerHistory(selectedDate || undefined);
  if (!j?.ok && !Array.isArray(j?.days)) return;
  if (Array.isArray(j.days) && j.days.length) weekDays = j.days;
  if (j.date) {
    selectedDate = j.date;
    const parts = String(j.date).split('-');
    dayLabel = parts.length === 3 ? `${parts[1]}-${parts[2]}` : j.date;
  } else if (j.today && !selectedDate) {
    selectedDate = j.today;
  }
  viewingToday = j.is_today === true || (!!j.today && selectedDate === j.today);
  if (Number.isFinite(j.day_start_ms) && Number.isFinite(j.day_end_ms)) {
    dayStartMs = j.day_start_ms;
    dayEndMs = j.day_end_ms;
  }
  const pts = j.ok && Array.isArray(j.points) ? j.points : [];
  powerHist.length = 0;
  for (const p of pts) {
    const t = Number(p.t);
    if (!Number.isFinite(t)) continue;
    powerHist.push({
      t,
      pct: Number(p.pct),
      volt: Number(p.volt),
    });
  }
  renderWeekBar();
  updateChartDayLabels();
}

/** Today 00:00–24:00 axis; 2h ticks (12 marks); plot samples at their real time. */
function drawSeriesChart(canvas, series, opts) {
  const sized = resizeCanvas(canvas);
  if (!sized) return;
  const { ctx, w, h } = sized;
  const padL = 32;
  const padR = 8;
  const padT = 10;
  const padB = 18;
  ctx.clearRect(0, 0, w, h);

  const plotW = w - padL - padR;
  const plotH = h - padT - padB;
  const minY = opts.min;
  const maxY = opts.max;
  const color = opts.color;
  const key = opts.key;
  const unit = opts.unit || '';
  const t0 = dayStartMs || Date.now() - DAY_MS;
  const span = Math.max(1, (dayEndMs || t0 + DAY_MS) - t0);

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

  // 12 ticks: 00,02,...,22
  ctx.fillStyle = C_MUTED;
  ctx.font = '8px IBM Plex Mono, monospace';
  ctx.textBaseline = 'alphabetic';
  for (let hour = 0; hour < 24; hour += TICK_HOURS) {
    const x = padL + (plotW * hour) / 24;
    ctx.beginPath();
    ctx.strokeStyle = 'rgba(12, 127, 150, 0.08)';
    ctx.moveTo(x, padT);
    ctx.lineTo(x, padT + plotH);
    ctx.stroke();
    ctx.fillStyle = C_MUTED;
    ctx.textAlign = hour === 0 ? 'left' : 'center';
    ctx.fillText(String(hour).padStart(2, '0'), x, h - 3);
  }

  if (!series.length) {
    ctx.fillStyle = C_MUTED;
    ctx.font = '12px IBM Plex Mono, monospace';
    ctx.textAlign = 'left';
    ctx.fillText(`${viewingToday ? '今日' : dayLabel || '该日'}暂无采样`, padL + 4, padT + plotH / 2);
    return;
  }

  const xAtT = (t) => padL + (plotW * Math.max(0, Math.min(1, (t - t0) / span)));
  const yAt = (v) => {
    const u = (v - minY) / (maxY - minY || 1);
    return padT + plotH * (1 - Math.max(0, Math.min(1, u)));
  };

  ctx.beginPath();
  let started = false;
  series.forEach((p) => {
    if (p.t < t0 || p.t > t0 + span) return;
    const x = xAtT(p.t);
    const y = yAt(Number(p[key]));
    if (!started) {
      ctx.moveTo(x, y);
      started = true;
    } else {
      ctx.lineTo(x, y);
    }
  });
  if (started) {
    ctx.strokeStyle = color;
    ctx.lineWidth = 2.2;
    ctx.stroke();
    if (series.length > 1) {
      const first = series.find((p) => p.t >= t0 && p.t <= t0 + span);
      const last = [...series].reverse().find((p) => p.t >= t0 && p.t <= t0 + span);
      if (first && last) {
        ctx.lineTo(xAtT(last.t), padT + plotH);
        ctx.lineTo(xAtT(first.t), padT + plotH);
        ctx.closePath();
        ctx.fillStyle = opts.fill || 'rgba(12, 127, 150, 0.12)';
        ctx.fill();
      }
    }
  }

  if (!lastPower.available && opts.showSim) {
    ctx.fillStyle = C_MUTED;
    ctx.font = '10px IBM Plex Mono, monospace';
    ctx.textAlign = 'right';
    ctx.fillText('NO BMS', padL + plotW, padT + 8);
  }

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

function formatEta(min) {
  const m = Math.round(min);
  const h = Math.floor(m / 60);
  const mm = m % 60;
  return h >= 1 ? `${h}h${mm}m` : `${mm}m`;
}

function applyEtaUi(pw) {
  const el = document.getElementById('dPowerEta');
  if (!el) return;
  const value = el.querySelector('.eta-value');
  if (!pw || !pw.available || !pw.charging) {
    el.hidden = true;
    return;
  }
  el.hidden = false;
  if (Number.isFinite(pw.etaMin) && pw.etaMin > 0) {
    value.textContent = formatEta(pw.etaMin);
    value.classList.remove('is-muted');
  } else if (pw.pct >= 99.5) {
    value.textContent = '即将充满';
    value.classList.add('is-muted');
  } else {
    value.textContent = '计算中…';
    value.classList.add('is-muted');
  }
}

function applyPowerUi(pw) {
  lastPower = pw;
  const el = document.getElementById('dPower');
  if (!pw.available) {
    el.innerHTML = '—';
    document.getElementById('dPowerSub').textContent = '等待 BMS 数据';
    tone(document.getElementById('kpiPower'), 'is-neutral');
    applyEtaUi(pw);
    if (viewingToday) {
      const nowPct = document.getElementById('chartNowPct');
      if (nowPct) nowPct.textContent = '当前 —';
      const nowVolt = document.getElementById('chartNowVolt');
      if (nowVolt) nowVolt.textContent = '当前 —';
    }
    return;
  }
  el.innerHTML = `${pw.pct.toFixed(0)}%`;
  const chg = pw.charging ? '充电中' : pw.docked ? '已对接' : '放电';
  const cur = pw.charging && pw.current > 0.01 ? ` · ${pw.current.toFixed(2)}A` : '';
  document.getElementById('dPowerSub').textContent = `${pw.volt.toFixed(1)}V · ${chg}${cur}`;
  tone(document.getElementById('kpiPower'), pw.pct >= 40 ? 'is-ok' : pw.pct >= 20 ? 'is-warn' : 'is-bad');
  applyEtaUi(pw);

  if (viewingToday) {
    const nowPct = document.getElementById('chartNowPct');
    if (nowPct) nowPct.textContent = `当前 ${pw.pct.toFixed(0)}%`;
    const nowVolt = document.getElementById('chartNowVolt');
    if (nowVolt) nowVolt.textContent = `当前 ${pw.volt.toFixed(1)}V`;
  } else {
    const last = powerHist.length ? powerHist[powerHist.length - 1] : null;
    const nowPct = document.getElementById('chartNowPct');
    const nowVolt = document.getElementById('chartNowVolt');
    if (nowPct) nowPct.textContent = last ? `当日 ${Number(last.pct).toFixed(0)}%` : '当日 —';
    if (nowVolt) nowVolt.textContent = last ? `当日 ${Number(last.volt).toFixed(1)}V` : '当日 —';
  }
}

let lastObstacle = null;

function renderCharts() {
  applyPowerUi(resolvePower(lastState?.power));

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

  const batt = lastPower.available ? lastPower.pct : 0;
  const locCode = Number(lastState?.localization_status ?? (lastState?.localization_ok ? 0 : 1));
  const locPts = locCode === 0 ? 30 : locCode === 2 ? 15 : 5;
  const health =
    (lastState?.safety_ok !== false ? 35 : 0) +
    (!lastState?.emergency_stop ? 35 : 0) +
    locPts;

  const gBatt = document.getElementById('gaugeBatt');
  const gHealth = document.getElementById('gaugeHealth');
  if (gBatt) {
    if (lastPower.available) {
      drawGauge(
        gBatt,
        batt,
        100,
        `${batt.toFixed(0)}%`,
        batt >= 40 ? '#0f8a5a' : batt >= 20 ? '#a37818' : '#c23145',
      );
    } else {
      drawGauge(gBatt, 0, 100, '—', C_MUTED);
    }
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
let histTimer = null;
let resizeHandler = null;
let weekBarHandler = null;
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
    applyPowerUi(resolvePower(s.power));
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
  const weekBar = document.getElementById('powerWeekBar');
  if (weekBar) {
    weekBarHandler = onWeekBarClick;
    weekBar.addEventListener('click', weekBarHandler);
  }
  loadPowerHistory().then(() => renderCharts());
  updateClock();
  clockTimer = setInterval(updateClock, 1000);
  // Gauges/radar stay 1s; history points only grow ~every 20s on server.
  chartTimer = setInterval(renderCharts, 1000);
  histTimer = setInterval(() => {
    if (!viewingToday) return;
    loadPowerHistory().then(() => renderCharts());
  }, 20000);
  sensorTimer = setInterval(refreshSensors, 5000);
  refreshSensors();
  renderCharts();
  resizeHandler = () => renderCharts();
  window.addEventListener('resize', resizeHandler, { passive: true });
  pushLog('总览就绪 · 电量可按本周日期切换（服务端落盘）', 'ok');
  return unmount;
}

export function unmount() {
  if (clockTimer) clearInterval(clockTimer);
  if (chartTimer) clearInterval(chartTimer);
  if (histTimer) clearInterval(histTimer);
  if (sensorTimer) clearInterval(sensorTimer);
  if (resizeHandler) window.removeEventListener('resize', resizeHandler);
  const weekBar = document.getElementById('powerWeekBar');
  if (weekBar && weekBarHandler) weekBar.removeEventListener('click', weekBarHandler);
  clockTimer = chartTimer = histTimer = sensorTimer = resizeHandler = weekBarHandler = null;
  if (metaHandler) offMeta(metaHandler);
  if (stateHandler) offState(stateHandler);
  if (obstacleHandler) offObstacle(obstacleHandler);
  if (taskHandler) offTask(taskHandler);
  metaHandler = stateHandler = obstacleHandler = taskHandler = null;
}
