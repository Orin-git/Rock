/**
 * Gen2 navigation page — map + lidar, initial pose drag, yaw dial, waypoint edit.
 */
import {
  connect,
  setMode,
  mapManage,
  waypointManage,
  publishGoal,
  publishInitialPose,
  startPatrol,
  cancelNav,
  publishTeleop,
  onTask,
  onState,
} from '/js/api.js';
import '/js/app.js';

connect();

const $ = (id) => document.getElementById(id);

const mapSelect = $('mapSelect');
const mapStatus = $('mapStatus');
const modeHint = $('modeHint');
const poseHint = $('poseHint');
const toolHint = $('toolHint');
const sessionHint = $('sessionHint');
const wpHint = $('wpHint');
const wpList = $('wpList');
const wpCount = $('wpCount');
const navFlash = $('navFlash');
const applyBtn = $('applyInitialPose');
const patrolLoop = $('patrolLoop');

let navActive = false;
let waypoints = [];
let selectedWp = null;
let selectedWpIdx = null;
let poseTimer = null;
let orientationControls = null;
let flashTimer = null;

const ORIENTATION_ROTATE_SPEED = Math.PI / 2;

function flash(msg, kind = 'info', ms = 5000) {
  if (!navFlash) return;
  navFlash.hidden = false;
  navFlash.className = `nav-flash is-${kind}`;
  navFlash.textContent = msg;
  if (flashTimer) clearTimeout(flashTimer);
  if (ms > 0) {
    flashTimer = setTimeout(() => {
      navFlash.hidden = true;
    }, ms);
  }
}

function pushLog(line) {
  const text = String(line || '').trim();
  if (!text) return;
  if (text.startsWith('!!') || /失败|failed|拒绝/.test(text)) {
    flash(text.replace(/^!!\s*/, ''), 'err');
  } else if (text.startsWith('<<') || text.includes('已保存') || text.includes('已删除')) {
    flash(text.replace(/^<<\s*/, ''), 'ok');
  } else if (text.startsWith('>>')) {
    flash(text.replace(/^>>\s*/, ''), 'info', 4500);
  } else {
    flash(text, 'info', 3500);
  }
}

function sleep(ms) {
  return new Promise((r) => setTimeout(r, ms));
}

function radToDeg(rad) {
  let d = ((Number(rad) || 0) * 180) / Math.PI;
  d = ((d % 360) + 360) % 360;
  return d;
}

function degToRad(deg) {
  return ((Number(deg) || 0) * Math.PI) / 180;
}

/** Gen1-style orientation slider + hold rotate (for initial pose / waypoint). */
class OrientationControls {
  constructor(container, opts) {
    opts = opts || {};
    this._getYaw = opts.getYaw || (() => 0);
    this._setYaw = opts.setYaw || (() => {});
    this._onChange = opts.onChange || null;
    this._rotFrame = null;
    this._rotDirection = 0;
    this._onGlobalPointerUp = this._stopRotate.bind(this);

    this._root = document.createElement('div');
    this._root.className = 'orientation-controls';
    this._root.innerHTML =
      '<div class="orientation-controls-header">' +
      '  <span class="orientation-controls-title">朝向控制</span>' +
      '  <span class="orientation-controls-value">0°</span>' +
      '</div>' +
      '<div class="orientation-slider-wrap">' +
      '  <input type="range" class="orientation-slider" min="0" max="360" step="0.5" value="0">' +
      '</div>' +
      '<div class="orientation-btn-row">' +
      '  <button type="button" class="orientation-rotate-btn orientation-rotate-ccw">' +
      '    <span>↺ 逆时针</span></button>' +
      '  <button type="button" class="orientation-rotate-btn orientation-rotate-cw">' +
      '    <span>↻ 顺时针</span></button>' +
      '</div>';
    container.appendChild(this._root);

    this._valueEl = this._root.querySelector('.orientation-controls-value');
    this._slider = this._root.querySelector('.orientation-slider');
    this._btnCcw = this._root.querySelector('.orientation-rotate-ccw');
    this._btnCw = this._root.querySelector('.orientation-rotate-cw');
    this._bindEvents();
    this.syncFromYaw();
  }

  syncFromYaw() {
    if (!this._slider || !this._valueEl) return;
    const deg = radToDeg(this._getYaw());
    this._slider.value = deg.toFixed(1);
    this._valueEl.textContent = Math.round(deg) + '°';
  }

  _applyYaw(yawRad) {
    this._setYaw(yawRad);
    this.syncFromYaw();
    if (this._onChange) {
      try {
        this._onChange(yawRad);
      } catch (_) {
        /* ignore */
      }
    }
  }

  _bindEvents() {
    const self = this;
    this._slider.addEventListener('input', () => {
      const deg = parseFloat(self._slider.value);
      if (!Number.isNaN(deg)) self._applyYaw(degToRad(deg));
    });
    const bindRotate = (btn, direction) => {
      const start = (e) => {
        e.preventDefault();
        e.stopPropagation();
        self._startRotate(direction, btn);
      };
      btn.addEventListener('mousedown', start);
      btn.addEventListener('touchstart', start, { passive: false });
    };
    bindRotate(this._btnCcw, 1);
    bindRotate(this._btnCw, -1);
  }

  _startRotate(direction, activeBtn) {
    this._stopRotate();
    this._rotDirection = direction;
    if (activeBtn) activeBtn.classList.add('active');
    document.addEventListener('mouseup', this._onGlobalPointerUp);
    document.addEventListener('touchend', this._onGlobalPointerUp);
    let lastTime = performance.now();
    const tick = (now) => {
      if (!this._rotDirection) return;
      const dt = Math.min((now - lastTime) / 1000, 0.05);
      lastTime = now;
      this._applyYaw(this._getYaw() + this._rotDirection * ORIENTATION_ROTATE_SPEED * dt);
      this._rotFrame = requestAnimationFrame(tick);
    };
    this._rotFrame = requestAnimationFrame(tick);
  }

  _stopRotate() {
    this._rotDirection = 0;
    if (this._rotFrame) {
      cancelAnimationFrame(this._rotFrame);
      this._rotFrame = null;
    }
    if (this._btnCcw) this._btnCcw.classList.remove('active');
    if (this._btnCw) this._btnCw.classList.remove('active');
    document.removeEventListener('mouseup', this._onGlobalPointerUp);
    document.removeEventListener('touchend', this._onGlobalPointerUp);
  }

  setVisible(on) {
    const panel = $('orientation-controls-panel');
    if (panel) panel.style.display = on ? 'block' : 'none';
  }
}

onTask((l) => {
  /* keep quiet on page; important actions use flash() */
  void l;
});
onState((s) => {
  const name = s.mode_name || String(s.mode);
  modeHint.textContent = `模式：${name} (${s.mode}) · ${s.detail || ''}`;
  navActive = Number(s.mode) === 2;
  sessionHint.textContent = navActive
    ? '导航中 · 拖设初位姿后点「确认初位姿」'
    : '选地图 → 进入导航 → 拖设初位姿并确认';
});

function currentMapName() {
  return (mapSelect.value || '').trim();
}

function syncToolButtons(mode) {
  document.querySelectorAll('.tool-btn').forEach((b) => b.classList.remove('active-tool'));
  const id = mode === 'initial_pose' ? 'toolInitialPose' : mode === 'edit_wp' ? 'toolEditWp' : null;
  if (id) {
    const el = $(id);
    if (el) el.classList.add('active-tool');
  }
  const labels = {
    view: '浏览（点列表/地图航点可前往）',
    initial_pose: '拖设初位姿',
    edit_wp: '编辑航点',
  };
  toolHint.textContent = `工具：${labels[mode] || mode}`;
  if (orientationControls) {
    orientationControls.setVisible(mode === 'initial_pose' || mode === 'edit_wp');
    orientationControls.syncFromYaw();
  }
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
    return false;
  }
  pushLog(`>> 加载地图 ${name}`);
  const j = await mapManage(5, name);
  if (!j.ok) {
    pushLog(`!! ${j.message || '加载失败'}`);
    return false;
  }
  let payload;
  try {
    payload = typeof j.data_json === 'string' ? JSON.parse(j.data_json || '{}') : j.data_json;
  } catch (_) {
    pushLog('!! 地图 JSON 无效');
    return false;
  }
  if (window.XwMapCanvas && window.XwMapCanvas.loadStaticMap(payload)) {
    pushLog(`<< 地图就绪 ${name}`);
    await loadWaypoints();
    return true;
  }
  return false;
}

async function loadWaypoints() {
  const name = currentMapName();
  waypoints = [];
  selectedWp = null;
  selectedWpIdx = null;
  $('gotoSelectedWp').disabled = true;
  $('deleteWp').disabled = true;
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
  waypoints = list
    .map((w, i) => ({
      name: w.name || w.id || `wp_${i + 1}`,
      x: Number(w.x),
      y: Number(w.y),
      yaw: Number(w.yaw != null ? w.yaw : w.theta != null ? w.theta : 0),
      _kind: String(w.name || '').toLowerCase() === 'charger' ? 'charger' : 'waypoint',
    }))
    .filter((w) => Number.isFinite(w.x) && Number.isFinite(w.y));

  if (window.XwMapCanvas) {
    window.XwMapCanvas.setWaypoints(waypoints);
    waypoints = window.XwMapCanvas.getWaypoints();
  }
  wpCount.textContent = String(waypoints.length);
  renderWpList();
}

function renderWpList() {
  wpList.innerHTML = '';
  if (!waypoints.length) {
    wpList.innerHTML = '<p class="muted pad">无航点 · 点「编辑航点」后在地图空白处点击打点</p>';
    return;
  }
  waypoints.forEach((wp, idx) => {
    const row = document.createElement('div');
    const bad = !!wp.bad;
    const selected = selectedWpIdx === idx || selectedWp === wp.name;
    row.className =
      'map-item nav-wp-item' + (selected ? ' selected' : '') + (bad ? ' wp-bad' : '');
    row.innerHTML = `
      <div class="map-item-main">
        <span class="index-badge">${bad ? '!' : idx + 1}</span>
        <div>
          <div class="map-item-name">${escapeHtml(wp.name)}${
            wp._kind === 'charger' ? ' · 充电桩' : ''
          }${bad ? ' · <span class="bad-tag">坏点</span>' : ''}</div>
          <div class="map-item-meta mono">x=${wp.x.toFixed(2)} y=${wp.y.toFixed(2)} yaw=${wp.yaw.toFixed(2)}</div>
        </div>
      </div>`;
    row.onclick = () => {
      selectWaypoint(idx, false);
      if (navActive && !bad) goToWaypoint(idx);
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

function selectWaypoint(idx, fromMap) {
  if (idx == null || !waypoints[idx]) return;
  const wp = waypoints[idx];
  selectedWpIdx = idx;
  selectedWp = wp.name;
  $('gotoSelectedWp').disabled = !!wp.bad;
  $('deleteWp').disabled = false;
  if (window.XwMapCanvas) {
    window.XwMapCanvas.setSelectedWaypointIndex(idx);
    if (!fromMap) window.XwMapCanvas.setGoal({ x: wp.x, y: wp.y, yaw: wp.yaw });
  }
  if (orientationControls) orientationControls.syncFromYaw();
  renderWpList();
}

async function goToWaypoint(idx) {
  const wp = waypoints[idx != null ? idx : selectedWpIdx];
  if (!wp) {
    flash('请先选中航点', 'err');
    return;
  }
  if (wp.bad) {
    flash('坏点不可前往（距障碍 <0.3m）', 'err');
    return;
  }
  if (!navActive) {
    flash('尚未进入导航，请先「进入导航」并确认初位姿', 'err');
    return;
  }
  if (window.XwMapCanvas) window.XwMapCanvas.setGoal({ x: wp.x, y: wp.y, yaw: wp.yaw });
  const j = await publishGoal(wp.x, wp.y, wp.yaw || 0, 'map');
  flash(
    j.ok
      ? `已前往 ${wp.name}（${wp.x.toFixed(2)}, ${wp.y.toFixed(2)}）`
      : `前往失败：${j.message || ''}`,
    j.ok ? 'ok' : 'err',
  );
}

function tickPose() {
  if (!window.XwMapCanvas) return;
  const p = window.XwMapCanvas.getRobotPose();
  if (!p) {
    poseHint.textContent = '位姿：等待 map→base（请先拖设初位姿）';
    return;
  }
  poseHint.textContent = `位姿：x=${p.x.toFixed(2)} y=${p.y.toFixed(2)} yaw=${p.yaw.toFixed(2)}`;
}

function syncInputsFromCanvasMode(_mode, state) {
  if (!state) return;
  if (Array.isArray(state.waypoints)) {
    waypoints = state.waypoints;
    wpCount.textContent = String(waypoints.length);
    if (state.selectedWpIdx != null && waypoints[state.selectedWpIdx]) {
      selectedWpIdx = state.selectedWpIdx;
      selectedWp = waypoints[state.selectedWpIdx].name;
      $('deleteWp').disabled = false;
      $('gotoSelectedWp').disabled = !!waypoints[state.selectedWpIdx].bad;
    }
    renderWpList();
  }
  if (orientationControls) orientationControls.syncFromYaw();
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
  // Load static map FIRST so the canvas isn't blank while Nav2 starts.
  const ok = await loadMapPreview();
  if (!ok) {
    pushLog('!! 地图加载失败，仍尝试进入导航');
  }
  pushLog(`>> 进入导航 setMode(2) map=${name}`);
  await setMode(2, { map_name: name });
  if (window.XwMapCanvas) {
    window.XwMapCanvas.enableLiveMap();
    window.XwMapCanvas.setInteractMode('initial_pose');
    syncToolButtons('initial_pose');
  }
  await loadWaypoints();
};

$('stopNav').onclick = async () => {
  pushLog('>> 结束导航 setMode(0)');
  await cancelNav();
  await setMode(0, {});
  if (window.XwMapCanvas) {
    window.XwMapCanvas.setInteractMode('view');
    window.XwMapCanvas.clearInitialPose();
    window.XwMapCanvas.clearGoal();
    syncToolButtons('view');
  }
};

$('estop').onclick = () => {
  publishTeleop(0, 0);
  cancelNav();
  pushLog('>> 急停：teleop 归零 + cancel');
};

function toggleTool(mode) {
  if (!window.XwMapCanvas) return;
  const cur = window.XwMapCanvas.getInteractMode();
  const next = cur === mode ? 'view' : mode;
  window.XwMapCanvas.setInteractMode(next);
  syncToolButtons(next);
  if (next === 'initial_pose') {
    pushLog('>> 拖设初位姿：拖动蓝点，右上角调朝向，再点「确认初位姿」');
  } else if (next === 'edit_wp') {
    pushLog('>> 编辑航点：空白处点击打点，拖动移动；改完请点「保存航点」');
  }
}

$('toolInitialPose').onclick = () => toggleTool('initial_pose');
$('toolEditWp').onclick = () => toggleTool('edit_wp');

$('applyInitialPose').onclick = async () => {
  if (!window.XwMapCanvas) {
    flash('画布未就绪', 'err');
    return;
  }
  if (window.XwMapCanvas.getInteractMode() !== 'initial_pose') {
    window.XwMapCanvas.setInteractMode('initial_pose');
    syncToolButtons('initial_pose');
  }
  const pose = window.XwMapCanvas.getInitialPose();
  if (!pose) {
    flash('请先在地图上拖设初位姿（点「拖设初位姿」后拖动蓝点）', 'err', 7000);
    return;
  }
  if (!navActive) {
    flash('尚未进入导航模式：请先点「进入导航」，再确认初位姿', 'err', 7000);
    return;
  }

  applyBtn.disabled = true;
  applyBtn.textContent = '发送中…';
  flash(
    `正在发布初位姿 x=${pose.x.toFixed(2)} y=${pose.y.toFixed(2)} yaw=${pose.yaw.toFixed(2)} …`,
    'info',
    0,
  );

  let last = { ok: false };
  try {
    for (let i = 0; i < 3; i++) {
      last = await publishInitialPose(pose.x, pose.y, pose.yaw, 'map');
      if (!last.ok) break;
      await sleep(120);
    }
  } finally {
    applyBtn.disabled = false;
    applyBtn.textContent = '确认初位姿';
  }

  if (last && last.ok) {
    flash(
      `初位姿已发送成功（x=${pose.x.toFixed(2)}, y=${pose.y.toFixed(2)}, yaw=${pose.yaw.toFixed(2)}）。请看地图上机器人是否跳到该位置。`,
      'ok',
      9000,
    );
    if (poseHint) poseHint.textContent = '初位姿已发 · 等待 AMCL 收敛';
    if (sessionHint) sessionHint.textContent = '初位姿已确认 · 可点航点前往或巡航';
    window.XwMapCanvas.setInteractMode('view');
    syncToolButtons('view');
  } else {
    flash(`初位姿发送失败：${(last && last.message) || '网络/桥接异常'}`, 'err', 8000);
  }
};

$('startPatrol').onclick = async () => {
  const name = currentMapName();
  if (!waypoints.length) {
    flash('没有航点，请先编辑并保存航点', 'err');
    return;
  }
  const loop = !!(patrolLoop && patrolLoop.checked);
  const j = await startPatrol({ map_name: name, loop });
  pushLog(j.ok ? `<< 多点巡航已启动${loop ? '（循环）' : ''}` : `!! ${j.message || 'failed'}`);
};

$('cancelNav').onclick = async () => {
  const j = await cancelNav();
  pushLog(j.ok ? '<< 导航已取消' : `!! ${j.message || 'failed'}`);
};

$('gotoSelectedWp').onclick = () => goToWaypoint(selectedWpIdx);

$('deleteWp').onclick = () => {
  if (!window.XwMapCanvas) return;
  if (window.XwMapCanvas.deleteSelectedWaypoint()) {
    waypoints = window.XwMapCanvas.getWaypoints();
    selectedWp = null;
    selectedWpIdx = null;
    $('deleteWp').disabled = true;
    $('gotoSelectedWp').disabled = true;
    wpCount.textContent = String(waypoints.length);
    renderWpList();
    pushLog('<< 已删除选中航点（未写入文件，请点「保存航点」）');
  }
};

$('saveWp').onclick = async () => {
  const name = currentMapName();
  if (!name) {
    flash('请先选择地图', 'err');
    return;
  }
  if (!window.XwMapCanvas) return;
  const list = window.XwMapCanvas.getWaypoints();
  const bad = list.filter((w) => w.bad);
  if (bad.length) {
    flash(`存在 ${bad.length} 个坏点（<0.3m 障碍），请先挪开或删除再保存`, 'err', 7000);
    return;
  }
  const payload = {
    waypoints: list.map((w) => ({
      name: w._kind === 'charger' ? 'charger' : w.name,
      x: w.x,
      y: w.y,
      yaw: w.yaw || 0,
    })),
  };
  const j = await waypointManage(1, name, { data_json: JSON.stringify(payload) });
  pushLog(j.ok ? `<< 航点已保存到地图「${name}」` : `!! ${j.message || 'save failed'}`);
  if (j.ok) await loadWaypoints();
};

if (window.XwMapCanvas) {
  const panel = $('orientation-controls-panel');
  orientationControls = new OrientationControls(panel, {
    getYaw: () => window.XwMapCanvas.getActiveYaw(),
    setYaw: (y) => {
      window.XwMapCanvas.setActiveYaw(y);
    },
    onChange: () => {},
  });
  orientationControls.setVisible(false);

  window.XwMapCanvas.start({
    connectionName: 'xw-nav-canvas',
    preferLiveMap: true,
    interactive: true,
    showSensorFrames: false,
    onStatus: (msg) => {
      mapStatus.textContent = msg;
    },
    onModeChange: (mode, state) => {
      syncToolButtons(mode);
      syncInputsFromCanvasMode(mode, state);
    },
    onYawChange: () => {
      if (orientationControls) orientationControls.syncFromYaw();
    },
    onWaypointClick: (wp, idx) => {
      selectWaypoint(idx, true);
      if (window.XwMapCanvas.getInteractMode() === 'view') {
        goToWaypoint(idx);
      }
    },
  });
  syncToolButtons('view');
} else {
  mapStatus.textContent = 'map_canvas.js 未加载';
}

poseTimer = setInterval(tickPose, 500);
refreshMaps();
