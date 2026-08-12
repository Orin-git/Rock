/**
 * Gen2 navigation page — shell + lidar/depth pipeline wiring.
 * Nav2 planning still stub; goals already reach xw_nav_session.
 */
import {
  connect,
  setMode,
  mapManage,
  waypointManage,
  publishGoal,
  publishTeleop,
  fetchSensorHub,
  onTask,
  onState,
} from '/js/api.js';
import '/js/app.js';

connect();

const $ = (id) => document.getElementById(id);

const logEl = $('log');
const mapSelect = $('mapSelect');
const mapStatus = $('mapStatus');
const modeHint = $('modeHint');
const poseHint = $('poseHint');
const sessionHint = $('sessionHint');
const goalHint = $('goalHint');
const wpList = $('wpList');
const wpCount = $('wpCount');
const sensorCards = $('sensorCards');
const sensorLayout = $('sensorLayout');
const gx = $('gx');
const gy = $('gy');
const gyaw = $('gyaw');

let navActive = false;
let waypoints = [];
let selectedWp = null;
let poseTimer = null;

function pushLog(line) {
  logEl.textContent = line + '\n' + logEl.textContent;
}

onTask((l) => pushLog(l));
onState((s) => {
  const name = s.mode_name || String(s.mode);
  modeHint.textContent = `模式：${name} (${s.mode}) · ${s.detail || ''}`;
  navActive = Number(s.mode) === 2;
  sessionHint.textContent = navActive
    ? '导航会话中 · 可发送 /xw/goal_pose（Nav2 接入前为 stub 回执）'
    : '会话未启动 · 先选地图并「进入导航」';
  goalHint.textContent = navActive
    ? '点击地图或填写坐标后发送'
    : '需先进入导航会话后再发送目标';
});

function currentMapName() {
  return (mapSelect.value || '').trim();
}

async function refreshMaps() {
  const j = await mapManage(2);
  const maps = j.map_list || [];
  const prev = mapSelect.value;
  mapSelect.innerHTML = '';
  if (!maps.length) {
    const opt = document.createElement('option');
    opt.value = '';
    opt.textContent = '（暂无地图）';
    mapSelect.appendChild(opt);
    return;
  }
  maps.forEach((name) => {
    const opt = document.createElement('option');
    opt.value = name;
    opt.textContent = name;
    mapSelect.appendChild(opt);
  });
  if (prev && maps.includes(prev)) mapSelect.value = prev;
  await loadWaypoints();
}

async function loadMapPreview() {
  const name = currentMapName();
  if (!name) {
    pushLog('!! 请先选择地图');
    return;
  }
  pushLog(`>> 加载静态地图预览 ${name}`);
  const j = await mapManage(5, name);
  if (!j.ok) {
    pushLog(`!! ${j.message || '加载失败'}`);
    return;
  }
  let payload;
  try {
    payload = typeof j.data_json === 'string' ? JSON.parse(j.data_json || '{}') : j.data_json;
  } catch (_) {
    pushLog('!! 地图 JSON 无效');
    return;
  }
  if (window.XwMapCanvas && window.XwMapCanvas.loadStaticMap(payload)) {
    pushLog(`<< 预览就绪 ${name}`);
    await loadWaypoints();
  }
}

async function loadWaypoints() {
  const name = currentMapName();
  waypoints = [];
  selectedWp = null;
  $('gotoSelectedWp').disabled = true;
  if (!name) {
    wpList.innerHTML = '<p class="muted pad">未选择地图</p>';
    wpCount.textContent = '0';
    if (window.XwMapCanvas) window.XwMapCanvas.setWaypoints([]);
    return;
  }
  const j = await waypointManage(2, name);
  let data = {};
  try {
    data = typeof j.data_json === 'string' ? JSON.parse(j.data_json || '{}') : j.data_json || {};
  } catch (_) {
    data = {};
  }
  const list = Array.isArray(data.waypoints) ? data.waypoints : [];
  // charger may sit at top-level
  if (data.charger && typeof data.charger === 'object') {
    list.unshift({
      name: 'charger',
      x: Number(data.charger.x),
      y: Number(data.charger.y),
      yaw: Number(data.charger.yaw || data.charger.theta || 0),
      _kind: 'charger',
    });
  }
  waypoints = list
    .map((w, i) => ({
      name: w.name || w.id || `wp_${i + 1}`,
      x: Number(w.x),
      y: Number(w.y),
      yaw: Number(w.yaw != null ? w.yaw : w.theta != null ? w.theta : 0),
      _kind: w._kind || 'waypoint',
    }))
    .filter((w) => Number.isFinite(w.x) && Number.isFinite(w.y));

  wpCount.textContent = String(waypoints.length);
  if (window.XwMapCanvas) window.XwMapCanvas.setWaypoints(waypoints);
  renderWpList();
}

function renderWpList() {
  wpList.innerHTML = '';
  if (!waypoints.length) {
    wpList.innerHTML = '<p class="muted pad">无航点（建图保存时会写入 charger）</p>';
    return;
  }
  waypoints.forEach((wp, idx) => {
    const row = document.createElement('div');
    row.className = 'map-item nav-wp-item' + (selectedWp === wp.name ? ' selected' : '');
    row.innerHTML = `
      <div class="map-item-main">
        <span class="index-badge">${idx + 1}</span>
        <div>
          <div class="map-item-name">${escapeHtml(wp.name)}${wp._kind === 'charger' ? ' · 充电桩' : ''}</div>
          <div class="map-item-meta mono">x=${wp.x.toFixed(2)} y=${wp.y.toFixed(2)} yaw=${wp.yaw.toFixed(2)}</div>
        </div>
      </div>`;
    row.onclick = () => {
      selectedWp = wp.name;
      gx.value = String(wp.x);
      gy.value = String(wp.y);
      gyaw.value = String(wp.yaw);
      if (window.XwMapCanvas) {
        window.XwMapCanvas.setGoal({ x: wp.x, y: wp.y, yaw: wp.yaw });
      }
      $('gotoSelectedWp').disabled = false;
      renderWpList();
    };
    wpList.appendChild(row);
  });
}

function escapeHtml(s) {
  return String(s)
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;');
}

function syncGoalInputsFromClick(world) {
  gx.value = world.x.toFixed(3);
  gy.value = world.y.toFixed(3);
  if (window.XwMapCanvas) {
    window.XwMapCanvas.setGoal({
      x: world.x,
      y: world.y,
      yaw: Number(gyaw.value) || 0,
    });
  }
}

async function sendGoalNow() {
  const x = Number(gx.value);
  const y = Number(gy.value);
  const yaw = Number(gyaw.value) || 0;
  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    pushLog('!! 坐标无效');
    return;
  }
  if (!navActive) {
    pushLog('!! 尚未进入导航模式（mode≠2），目标可能被 session 忽略');
  }
  if (window.XwMapCanvas) window.XwMapCanvas.setGoal({ x, y, yaw });
  const j = await publishGoal(x, y, yaw, 'map');
  pushLog(j.ok ? `<< goal published` : `!! ${j.message || 'goal failed'}`);
}

function statusClass(st) {
  if (st === 'live') return 'on';
  if (st === 'partial') return 'warn';
  if (st === 'placeholder') return 'off';
  return 'off';
}

function statusLabel(st) {
  if (st === 'live') return '在线';
  if (st === 'partial') return '部分';
  if (st === 'placeholder') return '占位';
  if (st === 'missing') return '未检出';
  return st || '—';
}

async function refreshSensors() {
  const j = await fetchSensorHub();
  const sensors = j.sensors || {};
  const order = ['lidar', 'depth_camera', 'ultrasonic', 'imu', 'chassis'];
  sensorCards.innerHTML = '';
  order.forEach((key) => {
    const s = sensors[key];
    if (!s) return;
    const card = document.createElement('div');
    card.className = 'sensor-card';
    const topics = (s.topics || []).map((t) => `<code>${escapeHtml(t)}</code>`).join(' ');
    card.innerHTML = `
      <div class="sensor-card-head">
        <strong>${escapeHtml(s.label || key)}</strong>
        <span class="pill ${statusClass(s.status)}">${statusLabel(s.status)}</span>
      </div>
      <div class="sensor-card-body muted">${topics || '—'}</div>
      ${s.hint ? `<div class="sensor-card-hint">${escapeHtml(s.hint)}</div>` : ''}
    `;
    sensorCards.appendChild(card);
  });

  // Live scan badge from canvas
  if (window.XwMapCanvas) {
    const scan = window.XwMapCanvas.getScanStatus();
    const lidarCard = sensorCards.querySelector('.sensor-card');
    if (lidarCard && scan.hasScan) {
      const extra = document.createElement('div');
      extra.className = 'sensor-card-hint';
      extra.textContent = scan.live
        ? `画布激光活跃 · ${scan.ranges} beams · ${scan.ageMs}ms`
        : `画布有扫描缓存 · 年龄 ${scan.ageMs}ms`;
      lidarCard.appendChild(extra);
    }
  }

  renderLayout(j.layout || []);
}

function renderLayout(layout) {
  if (!layout.length && window.XwMapCanvas) {
    layout = (window.XwMapCanvas.DEFAULT_SENSOR_FRAMES || []).map((f) => ({
      id: f.id,
      frame: f.frame,
      label: f.label,
      xyz: [0, 0, 0],
      status: 'placeholder',
    }));
  }
  // Top-down schematic: x forward (up on SVG), y left
  const W = 220;
  const H = 200;
  const cx = W / 2;
  const cy = H / 2 + 10;
  const scale = 180; // px per meter
  let dots = '';
  layout.forEach((s) => {
    const xyz = s.xyz || [0, 0, 0];
    const px = cx - xyz[1] * scale;
    const py = cy - xyz[0] * scale;
    const color =
      s.status === 'live' ? '#22c55e' : s.status === 'partial' ? '#f59e0b' : '#94a3b8';
    dots += `<circle cx="${px}" cy="${py}" r="6" fill="${color}" />
      <text x="${px}" y="${py - 10}" text-anchor="middle" class="layout-label">${escapeHtml(
      s.label || s.id
    )}</text>`;
  });
  sensorLayout.innerHTML = `
    <svg viewBox="0 0 ${W} ${H}" width="100%" height="180" role="img">
      <rect x="70" y="70" width="80" height="60" rx="8" fill="#1e293b" stroke="#64748b" />
      <text x="${cx}" y="105" text-anchor="middle" fill="#e2e8f0" font-size="11">base_link</text>
      <polygon points="${cx},55 ${cx - 8},68 ${cx + 8},68" fill="#38bdf8" />
      <text x="${cx}" y="48" text-anchor="middle" fill="#64748b" font-size="10">+X</text>
      ${dots}
    </svg>`;
}

function tickPose() {
  if (!window.XwMapCanvas) return;
  const p = window.XwMapCanvas.getRobotPose();
  if (!p) {
    poseHint.textContent = '位姿：等待 TF map→base_link';
    return;
  }
  poseHint.textContent = `位姿：x=${p.x.toFixed(2)} y=${p.y.toFixed(2)} yaw=${p.yaw.toFixed(2)}`;
}

// ——— wire UI ———
$('refreshMaps').onclick = () => refreshMaps();
$('loadMapPreview').onclick = () => loadMapPreview();
$('reloadWp').onclick = () => loadWaypoints();
$('mapSelect').onchange = () => loadWaypoints();

$('startNav').onclick = async () => {
  const name = currentMapName();
  if (!name) {
    alert('请先选择地图');
    return;
  }
  pushLog(`>> 进入导航 setMode(2) map=${name}`);
  await setMode(2, { map_name: name });
  // Prefer live /map when Nav2 map_server is up; keep static preview if not.
  if (window.XwMapCanvas) window.XwMapCanvas.enableLiveMap();
  await loadWaypoints();
};

$('stopNav').onclick = async () => {
  pushLog('>> 结束导航 setMode(0)');
  await setMode(0, {});
};

$('estop').onclick = () => {
  publishTeleop(0, 0);
  pushLog('>> 急停：teleop 归零');
};

$('sendGoal').onclick = () => sendGoalNow();
$('clearGoal').onclick = () => {
  if (window.XwMapCanvas) window.XwMapCanvas.clearGoal();
};
$('gotoSelectedWp').onclick = () => sendGoalNow();
$('refreshSensors').onclick = () => refreshSensors();

$('clickGoal').onchange = (ev) => {
  if (window.XwMapCanvas) {
    window.XwMapCanvas.setInteractive(!!ev.target.checked);
  }
};
$('showFrames').onchange = (ev) => {
  if (window.XwMapCanvas) {
    window.XwMapCanvas.setShowSensorFrames(!!ev.target.checked);
  }
};

if (window.XwMapCanvas) {
  window.XwMapCanvas.start({
    connectionName: 'xw-nav-canvas',
    preferLiveMap: true,
    interactive: true,
    showSensorFrames: false,
    onStatus: (msg) => {
      mapStatus.textContent = msg;
    },
    onMapClick: (world) => {
      if (!$('clickGoal').checked) return;
      syncGoalInputsFromClick(world);
      pushLog(`地图点击 → (${world.x.toFixed(2)}, ${world.y.toFixed(2)})`);
    },
  });
} else {
  mapStatus.textContent = 'map_canvas.js 未加载';
}

poseTimer = setInterval(tickPose, 500);
refreshMaps();
refreshSensors();
setInterval(refreshSensors, 8000);
