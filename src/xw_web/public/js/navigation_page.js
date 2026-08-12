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
  const order = ['lidar', 'depth_camera', 'depth_camera_2', 'ultrasonic', 'imu', 'chassis'];
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

function layoutStatusColor(status) {
  if (status === 'live') return '#22c55e';
  if (status === 'partial') return '#f59e0b';
  return '#94a3b8';
}

function shortSensorLabel(s) {
  const id = String(s.id || '');
  if (id === 'lidar') return '激光';
  if (id === 'camera_front') return '前视';
  if (id === 'camera_front_2') return '前视2';
  if (id === 'camera_rear') return '后视';
  if (id === 'ultrasonic') return '超声';
  if (id === 'imu') return 'IMU';
  if (id === 'chassis') return '底盘';
  const full = String(s.label || id);
  return full.length > 4 ? full.slice(0, 4) : full;
}

/**
 * Isometric transparent chassis + sensor markers (URDF relative to base_link).
 * ROS: +X forward, +Y left, +Z up.
 */
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

  const W = 380;
  const H = 300;
  const ox = W * 0.48;
  const oy = H * 0.62;
  const scale = 210; // px / m

  // Isometric: screen from front-right-above
  const cos30 = Math.cos(Math.PI / 6);
  const sin30 = Math.sin(Math.PI / 6);
  function project(x, y, z) {
    return {
      sx: ox + (x - y) * cos30 * scale,
      sy: oy - z * scale - (x + y) * sin30 * scale,
    };
  }
  function poly(pts) {
    return pts.map((p) => `${p.sx.toFixed(1)},${p.sy.toFixed(1)}`).join(' ');
  }

  // URDF chassis box 0.45 × 0.35 × 0.18 centered on base_link
  const hx = 0.225;
  const hy = 0.175;
  const hz = 0.09;
  const c = {
    // bottom
    b000: project(-hx, -hy, -hz),
    b100: project(hx, -hy, -hz),
    b110: project(hx, hy, -hz),
    b010: project(-hx, hy, -hz),
    // top
    t000: project(-hx, -hy, hz),
    t100: project(hx, -hy, hz),
    t110: project(hx, hy, hz),
    t010: project(-hx, hy, hz),
  };

  // Faces back-to-front for transparency
  const faces = [
    { pts: [c.b010, c.b110, c.t110, c.t010], fill: 'rgba(56,189,248,0.06)', stroke: '#475569' }, // +Y left
    { pts: [c.b000, c.b010, c.t010, c.t000], fill: 'rgba(148,163,184,0.08)', stroke: '#475569' }, // -X rear
    { pts: [c.b000, c.b100, c.b110, c.b010], fill: 'rgba(15,23,42,0.35)', stroke: '#64748b' }, // bottom
    { pts: [c.b100, c.b110, c.t110, c.t100], fill: 'rgba(56,189,248,0.12)', stroke: '#38bdf8' }, // +X front
    { pts: [c.b000, c.b100, c.t100, c.t000], fill: 'rgba(148,163,184,0.10)', stroke: '#64748b' }, // -Y right
    { pts: [c.t000, c.t100, c.t110, c.t010], fill: 'rgba(148,163,184,0.14)', stroke: '#94a3b8' }, // top
  ];

  let body = faces
    .map(
      (f) =>
        `<polygon points="${poly(f.pts)}" fill="${f.fill}" stroke="${f.stroke}" stroke-width="1.2" />`
    )
    .join('');

  // Vertical edge emphasis (wireframe feel)
  const edges = [
    [c.b000, c.t000],
    [c.b100, c.t100],
    [c.b110, c.t110],
    [c.b010, c.t010],
  ];
  body += edges
    .map(
      ([a, b]) =>
        `<line x1="${a.sx}" y1="${a.sy}" x2="${b.sx}" y2="${b.sy}" stroke="#64748b" stroke-width="1" stroke-opacity="0.7" />`
    )
    .join('');

  // Axis triad at base_link origin
  const o = project(0, 0, 0);
  const ax = project(0.16, 0, 0);
  const ay = project(0, 0.14, 0);
  const az = project(0, 0, 0.16);
  const axes = `
    <line x1="${o.sx}" y1="${o.sy}" x2="${ax.sx}" y2="${ax.sy}" stroke="#38bdf8" stroke-width="2" />
    <line x1="${o.sx}" y1="${o.sy}" x2="${ay.sx}" y2="${ay.sy}" stroke="#4ade80" stroke-width="2" />
    <line x1="${o.sx}" y1="${o.sy}" x2="${az.sx}" y2="${az.sy}" stroke="#c084fc" stroke-width="2" />
    <text x="${ax.sx + 4}" y="${ax.sy + 3}" class="layout-axis" fill="#38bdf8">X前</text>
    <text x="${ay.sx - 2}" y="${ay.sy - 4}" class="layout-axis" fill="#4ade80">Y左</text>
    <text x="${az.sx + 4}" y="${az.sy}" class="layout-axis" fill="#c084fc">Z上</text>
    <circle cx="${o.sx}" cy="${o.sy}" r="2.5" fill="#e2e8f0" />`;

  // Front direction chevron on top face
  const nose = project(hx + 0.04, 0, hz);
  const noseL = project(hx - 0.02, 0.04, hz);
  const noseR = project(hx - 0.02, -0.04, hz);
  const noseMark = `<polygon points="${poly([nose, noseL, noseR])}" fill="#38bdf8" opacity="0.9" />`;

  const items = layout.map((s) => {
    const xyz = s.xyz || [0, 0, 0];
    const x = Number(xyz[0]) || 0;
    const y = Number(xyz[1]) || 0;
    const z = Number(xyz[2]) || 0;
    const p = project(x, y, z);
    return {
      ...s,
      x,
      y,
      z,
      px: p.sx,
      py: p.sy,
      color: layoutStatusColor(s.status),
      short: shortSensorLabel(s),
    };
  });

  // Depth sort: draw farther first (higher screen-y is nearer in our iso)
  const sorted = [...items].sort((a, b) => a.py - b.py);

  // Callouts: aft / left-of-screen → left; forward / right → right; centerline split by z
  const left = [];
  const right = [];
  const mid = [];
  items.forEach((it) => {
    if (it.x < -0.05 || (Math.abs(it.x) <= 0.05 && it.y > 0.02)) left.push(it);
    else if (it.x > 0.05 || (Math.abs(it.x) <= 0.05 && it.y < -0.02)) right.push(it);
    else mid.push(it);
  });
  mid.sort((a, b) => b.z - a.z);
  mid.forEach((it, i) => (i % 2 === 0 ? left : right).push(it));
  left.sort((a, b) => a.py - b.py);
  right.sort((a, b) => a.py - b.py);

  const labelTop = 36;
  const labelBot = H - 36;
  function placeSide(sideItems, side) {
    const n = Math.max(sideItems.length, 1);
    const span = labelBot - labelTop;
    return sideItems.map((it, i) => {
      const ly = n === 1 ? (labelTop + labelBot) / 2 : labelTop + (span * i) / (n - 1);
      return {
        ...it,
        side,
        lx: side === 'left' ? 10 : W - 10,
        ly,
        elbowX: side === 'left' ? 92 : W - 92,
      };
    });
  }
  const callouts = [...placeSide(left, 'left'), ...placeSide(right, 'right')];

  let leaders = '';
  callouts.forEach((it) => {
    const name = escapeHtml(it.label || it.id);
    const xyzTxt = `(${it.x.toFixed(2)}, ${it.y.toFixed(2)}, ${it.z.toFixed(2)})`;
    const axText = it.side === 'left' ? it.lx : it.lx;
    const anchor = it.side === 'left' ? 'start' : 'end';
    leaders += `
      <path d="M ${it.px} ${it.py} L ${it.elbowX} ${it.ly} L ${axText} ${it.ly}"
        fill="none" stroke="${it.color}" stroke-width="1.15" stroke-opacity="0.75" />
      <text x="${axText}" y="${it.ly - 3}" text-anchor="${anchor}" class="layout-label">${name}</text>
      <text x="${axText}" y="${it.ly + 10}" text-anchor="${anchor}" class="layout-xyz">${xyzTxt}</text>`;
  });

  let marks = '';
  sorted.forEach((it) => {
    // Drop line to chassis top for height cue
    const foot = project(it.x, it.y, Math.min(it.z, hz));
    if (it.z > hz + 0.01) {
      marks += `<line x1="${foot.sx}" y1="${foot.sy}" x2="${it.px}" y2="${it.py}"
        stroke="${it.color}" stroke-width="1" stroke-dasharray="3 2" opacity="0.55" />`;
    }
    marks += `
      <circle cx="${it.px}" cy="${it.py}" r="6" fill="${it.color}" stroke="#0f172a" stroke-width="1.6" />
      <text x="${it.px}" y="${it.py - 10}" text-anchor="middle" class="layout-dot-label">${escapeHtml(
      it.short
    )}</text>`;
  });

  sensorLayout.innerHTML = `
    <svg viewBox="0 0 ${W} ${H}" width="100%" height="280" role="img" aria-label="三维透明机身与传感器位姿">
      <text x="${W / 2}" y="18" text-anchor="middle" class="layout-caption">透明机身 · URDF 相对 base_link</text>
      ${body}
      ${noseMark}
      ${axes}
      ${leaders}
      ${marks}
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
