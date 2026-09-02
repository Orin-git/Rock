/**
 * Gen2 navigation page — map + lidar, initial pose drag, yaw dial, waypoint edit.
 */
import {
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
  offTask,
  offState,
  fetchFollowStatus,
  setFollowEnabled,
  fetchRechargeStatus,
  setRechargeEnabled,
} from '/js/api.js';
const $ = (id) => document.getElementById(id);

function bindDom() {
  mapSelect = $('mapSelect');
  mapStatus = $('mapStatus');
  modeHint = $('modeHint');
  poseHint = $('poseHint');
  toolHint = $('toolHint');
  sessionHint = $('sessionHint');
  wpHint = $('wpHint');
  wpList = $('wpList');
  wpCount = $('wpCount');
  navFlash = $('navFlash');
  applyBtn = $('applyInitialPose');
  patrolLoop = $('patrolLoop');
  navFollowBtn = $('navFollowBtn');
  navRechargeBtn = $('navRechargeBtn');
  navTaskHint = $('navTaskHint');
  navLocChip = $('navLocChip');
  navLocBadge = $('navLocBadge');
  navLocText = $('navLocText');
  navGoalChip = $('navGoalChip');
  navGoalText = $('navGoalText');
  navGoalSub = $('navGoalSub');
  navRechargeStrip = $('navRechargeStrip');
  navRechargePhase = $('navRechargePhase');
  navRechargeMsg = $('navRechargeMsg');
}

let followTimer = null;
let taskHandler = null;
let stateHandlerA = null;
let stateHandlerB = null;

let mapSelect;
let mapStatus;
let modeHint;
let poseHint;
let toolHint;
let sessionHint;
let wpHint;
let wpList;
let wpCount;
let navFlash;
let applyBtn;
let patrolLoop;
let navFollowBtn;
let navRechargeBtn;
let navTaskHint;
let navLocChip;
let navLocBadge;
let navLocText;
let navGoalChip;
let navGoalText;
let navGoalSub;
let navRechargeStrip;
let navRechargePhase;
let navRechargeMsg;

let navActive = false;
let navGoalPhase = 'idle';
let navGoalDetail = '';
/** idle | pending (local goal sent) | active (saw start task) — avoids stale「到啦」tip. */
let navGoalCycle = 'idle';
let waypoints = [];
let selectedWp = null;
let selectedWpIdx = null;
/** Multi-select indices for batch delete (Set of numbers). */
let checkedWpIdx = new Set();
let poseTimer = null;
let orientationControls = null;
let flashTimer = null;
let followEnabled = false;
let followBusy = false;
let rechargeActive = false;
let rechargeBusy = false;
let lastRechargePhase = 'idle';
let lastFailHeld = '';
/** Last active_map we synced into the map select (avoid reload thrash). */
let syncedActiveMap = '';
const NAV_MAP_STORAGE_KEY = 'xw_nav_selected_map';
const LOC_LABELS = { 0: '正常', 1: '未就绪', 2: '漂移自愈', 3: '需重定位' };
const NAV_GOAL_LABELS = {
  idle: '待命',
  navigating: '导航中',
  patrol: '巡航中',
  following: '跟随中',
  arrived: '已到达',
  failed: '未到达',
  cancelled: '已取消',
};

function rememberMapName(name) {
  const n = String(name || '').trim();
  if (!n) return;
  try {
    sessionStorage.setItem(NAV_MAP_STORAGE_KEY, n);
  } catch (_) {
    /* ignore */
  }
}

function recalledMapName() {
  try {
    return (sessionStorage.getItem(NAV_MAP_STORAGE_KEY) || '').trim();
  } catch (_) {
    return '';
  }
}

/** When supervisor reports NAVIGATING, keep live /map and match active_map. */
function syncNavSessionFromState(s) {
  if (!s) return;
  const mode = Number(s.mode);
  const active = mode === 2 || mode === 3;
  if (active && window.XwMapCanvas) {
    window.XwMapCanvas.enableLiveMap();
  }
  const am = String(s.active_map || '').trim();
  if (!am || !mapSelect) return;
  const hasOpt = Array.from(mapSelect.options).some((o) => o.value === am);
  if (!hasOpt) return;
  if (am === syncedActiveMap && mapSelect.value === am) {
    return;
  }
  const firstSyncForMap = am !== syncedActiveMap;
  syncedActiveMap = am;
  rememberMapName(am);
  if (mapSelect.value !== am) {
    mapSelect.value = am;
    void loadMapPreview({ allowLive: active });
  } else if (active && firstSyncForMap && window.XwMapCanvas) {
    // Refresh while already on the correct map: re-pull latched /map once.
    window.XwMapCanvas.enableLiveMap({ forceResub: true });
  }
}

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

  destroy() {
    this._stopRotate();
    if (this._root?.parentNode) this._root.parentNode.removeChild(this._root);
    this._root = null;
  }
}


function applyLocSnapshot(s) {
  if (!navLocChip) return;
  const code = Number(s?.localization_status ?? (s?.localization_ok ? 0 : 1));
  const c = Number.isFinite(code) ? Math.max(0, Math.min(3, code)) : 1;
  navLocChip.dataset.loc = String(c);
  if (navLocBadge) navLocBadge.textContent = String(c);
  const label = LOC_LABELS[c] ?? `状态 ${c}`;
  if (navLocText) navLocText.textContent = label;
  navLocChip.title = `定位健康 ${c} · ${label}`;
}

function renderNavGoalChip() {
  if (!navGoalChip) return;
  navGoalChip.dataset.phase = navGoalPhase;
  const main = NAV_GOAL_LABELS[navGoalPhase] || '待命';
  if (navGoalText) navGoalText.textContent = main;
  if (navGoalSub) {
    navGoalSub.textContent = navGoalDetail && navGoalDetail !== main ? navGoalDetail : '';
  }
  navGoalChip.title = navGoalDetail || main;
}

function setNavGoalPhase(phase, detail = '') {
  navGoalPhase = phase || 'idle';
  navGoalDetail = detail ? String(detail) : '';
  renderNavGoalChip();
}

function applyNavGoalFromTask(text) {
  const t = String(text || '').trim();
  if (!t) return false;
  if (/跟着你|找你中/.test(t)) {
    navGoalCycle = 'idle';
    setNavGoalPhase('following', t);
    return true;
  }
  if (/不跟了/.test(t) && navActive) {
    navGoalCycle = 'idle';
    setNavGoalPhase('idle');
    return true;
  }
  // Terminal outcomes first — never let「进行中」win over a later「到啦」replay.
  if (/到啦|巡航走完了|导航好了/.test(t)) {
    navGoalCycle = 'idle';
    setNavGoalPhase('arrived', t);
    return true;
  }
  if (/没走到|巡航没走完|导航失败/.test(t)) {
    navGoalCycle = 'idle';
    setNavGoalPhase('failed', t);
    return true;
  }
  if (/导航取消|巡航停了|巡航取消了|导航已取消/.test(t)) {
    navGoalCycle = 'idle';
    setNavGoalPhase('cancelled', t);
    return true;
  }
  if (/导航：开始前往|开始前往|导航：进行中|正在前往|收到，开始动|去那边/.test(t)) {
    navGoalCycle = 'active';
    // Keep「前往 wp_x」subtitle when the immediate「去那边」ack arrives.
    if (
      navGoalPhase === 'navigating' &&
      /前往\s+\S+/.test(navGoalDetail) &&
      /^去那边$/.test(t)
    ) {
      return true;
    }
    setNavGoalPhase('navigating', t.replace(/^导航：/, ''));
    return true;
  }
  if (/开始巡航|巡航中|去下一个|巡航已启动|patrol/i.test(t)) {
    navGoalCycle = 'active';
    setNavGoalPhase('patrol', t.replace(/^导航：/, ''));
    return true;
  }
  return false;
}

function syncNavGoalFromState(s) {
  const mode = Number(s?.mode);
  if (mode === 3) {
    navGoalCycle = 'idle';
    setNavGoalPhase('following', '人体跟随');
    return;
  }
  if (mode !== 2) {
    navGoalCycle = 'idle';
    setNavGoalPhase('idle');
    return;
  }
  if (navGoalPhase === 'following') {
    setNavGoalPhase('idle');
  }
  // Mode still NAVIGATING after goal done: force chip from latest task tip.
  // Skip while pending — tip may still be the previous「到啦」.
  const tip = String(s?.latest_task || '').trim();
  if (!tip) return;
  const activeChip = navGoalPhase === 'navigating' || navGoalPhase === 'patrol';
  if (!activeChip) return;
  if (navGoalCycle === 'pending') {
    if (/开始前往|进行中|正在前往|去那边|开始巡航|巡航中|巡航已启动/.test(tip)) {
      applyNavGoalFromTask(tip);
    }
    return;
  }
  if (/到啦|巡航走完了|导航好了|没走到|巡航没走完|导航失败|导航取消|巡航停了|巡航取消了|导航已取消/.test(tip)) {
    applyNavGoalFromTask(tip);
  }
}

function wireNavigation(ctx) {
  bindDom();
  renderNavGoalChip();
  const queryMap = (ctx?.query?.get && ctx.query.get('map')) || new URLSearchParams(location.search).get('map');
  taskHandler = (line) => {
    applyNavGoalFromTask(line);
  };
  stateHandlerA = (s) => {
    const mode = Number(s.mode);
    const nameMap = { 0: '空闲', 1: '建图', 2: '导航', 3: '跟随', 4: '跌倒监测' };
    const name = nameMap[mode] || s.mode_name || String(s.mode);
    if (modeHint) modeHint.textContent = `模式：${name}`;
    navActive = mode === 2 || mode === 3;
    if (sessionHint) {
      sessionHint.textContent = navActive
        ? '导航中 · 拖设初位姿后点「确认」'
        : '选地图 → 进导航 → 拖设初位姿并确认';
    }
    applyLocSnapshot(s);
    syncNavGoalFromState(s);
    syncNavSessionFromState(s);
  };

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
    const params = new URLSearchParams(window.location.search || '');
    const fromQuery = (params.get('map') || '').trim();
    const fromStore = recalledMapName();
    const prev = mapSelect.value || fromQuery || fromStore;
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
    if (currentMapName()) {
      rememberMapName(currentMapName());
      await loadMapPreview({ allowLive: navActive });
    } else {
      await loadWaypoints();
    }
  }

  async function loadMapPreview(opts) {
    opts = opts || {};
    const name = currentMapName();
    if (!name) {
      pushLog('!! 请先选择地图');
      return false;
    }
    const allowLive = opts.allowLive === true || navActive;
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
    if (window.XwMapCanvas && window.XwMapCanvas.loadStaticMap(payload, { allowLive })) {
      rememberMapName(name);
      pushLog(`<< 地图就绪 ${name}`);
      if (allowLive) {
        window.XwMapCanvas.enableLiveMap({ forceResub: true });
      }
      await loadWaypoints();
      return true;
    }
    return false;
  }

  async function loadWaypoints() {
    const name = currentMapName();
    const wpMapTag = $('wpMapTag');
    waypoints = [];
    selectedWp = null;
    selectedWpIdx = null;
    checkedWpIdx = new Set();
    $('gotoSelectedWp').disabled = true;
    $('deleteWp').disabled = true;
    if (wpMapTag) wpMapTag.textContent = name ? `· ${name}_pointList` : '';
    const fileHint = $('wpListFileHint');
    if (fileHint) fileHint.textContent = name ? `${name}_pointList` : '{地图}_pointList';
    if (!name) {
      wpList.innerHTML = '<p class="muted pad">未选择地图（航点按地图分别保存）</p>';
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
    if (!j.ok) {
      wpList.innerHTML = `<p class="muted pad">加载失败：${escapeHtml(j.message || 'unknown')}</p>`;
      wpCount.textContent = '0';
      if (window.XwMapCanvas) window.XwMapCanvas.setWaypoints([]);
      return;
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

  function isChargerWaypoint(wp) {
    return !!(wp && (wp._kind === 'charger' || String(wp.name || '').toLowerCase() === 'charger'));
  }

  function isReservedWaypointName(name) {
    return String(name || '').trim().toLowerCase() === 'charger';
  }

  function syncWaypointsToCanvas() {
    if (!window.XwMapCanvas) return;
    window.XwMapCanvas.setWaypoints(waypoints);
    waypoints = window.XwMapCanvas.getWaypoints();
    if (selectedWpIdx != null && waypoints[selectedWpIdx]) {
      window.XwMapCanvas.setSelectedWaypointIndex(selectedWpIdx);
      selectedWp = waypoints[selectedWpIdx].name;
    }
  }

  function normalizeWaypointName(raw) {
    return String(raw || '')
      .trim()
      .replace(/\s+/g, '_');
  }

  function validateWaypointName(idx, raw) {
    const name = normalizeWaypointName(raw);
    if (!name) return { ok: false, message: '名称不能为空' };
    if (name.length > 48) return { ok: false, message: '名称过长（最多 48 字符）' };
    if (!/^[\w\u4e00-\u9fff\-./]+$/u.test(name)) {
      return { ok: false, message: '仅支持中英文、数字、_ - . /' };
    }
    if (isReservedWaypointName(name)) {
      return { ok: false, message: 'charger 为充电桩保留名，请换一个' };
    }
    const clash = waypoints.findIndex(
      (w, i) => i !== idx && String(w.name || '').toLowerCase() === name.toLowerCase(),
    );
    if (clash >= 0) return { ok: false, message: `名称「${name}」已被占用` };
    return { ok: true, name };
  }

  function commitWaypointRename(idx, raw) {
    const wp = waypoints[idx];
    if (!wp || isChargerWaypoint(wp)) return false;
    const checked = validateWaypointName(idx, raw);
    if (!checked.ok) {
      flash(checked.message, 'err');
      return false;
    }
    if (checked.name === wp.name) {
      renderWpList();
      return true;
    }
    wp.name = checked.name;
    if (selectedWpIdx === idx) selectedWp = checked.name;
    syncWaypointsToCanvas();
    renderWpList();
    pushLog(`<< 已重命名为「${checked.name}」（未写入文件，请点「保存航点」）`);
    return true;
  }

  function beginWaypointRename(idx) {
    const wp = waypoints[idx];
    if (!wp) return;
    if (isChargerWaypoint(wp)) {
      flash('充电桩名称固定为 charger，不可重命名', 'err');
      return;
    }
    selectWaypoint(idx, false);
    const row = wpList.querySelector(`.nav-wp-item[data-wp-idx="${idx}"]`);
    if (!row) return;
    const nameEl = row.querySelector('.nav-wp-name');
    if (!nameEl || nameEl.querySelector('input')) return;

    const chips = Array.from(nameEl.querySelectorAll('.nav-wp-chip'));
    nameEl.innerHTML = '';
    const input = document.createElement('input');
    input.type = 'text';
    input.className = 'nav-wp-rename-input';
    input.value = wp.name;
    input.maxLength = 48;
    input.setAttribute('aria-label', '重命名航点');
    nameEl.appendChild(input);
    chips.forEach((c) => nameEl.appendChild(c));
    input.focus();
    input.select();

    let done = false;
    const finish = (save) => {
      if (done) return;
      done = true;
      if (save) {
        if (!commitWaypointRename(idx, input.value)) {
          done = false;
          input.focus();
          input.select();
          return;
        }
        return;
      }
      renderWpList();
    };
    input.addEventListener('keydown', (ev) => {
      if (ev.key === 'Enter') {
        ev.preventDefault();
        ev.stopPropagation();
        finish(true);
      } else if (ev.key === 'Escape') {
        ev.preventDefault();
        ev.stopPropagation();
        finish(false);
      }
    });
    input.addEventListener('blur', () => finish(true));
    input.addEventListener('click', (ev) => ev.stopPropagation());
    input.addEventListener('pointerdown', (ev) => ev.stopPropagation());
  }

  function updateDeleteGotoButtons() {
    const hasChecked = checkedWpIdx.size > 0;
    const hasSingle = selectedWpIdx != null && waypoints[selectedWpIdx];
    $('deleteWp').disabled = !(hasChecked || hasSingle);
    if (hasSingle) {
      $('gotoSelectedWp').disabled = !!waypoints[selectedWpIdx].bad;
    } else {
      $('gotoSelectedWp').disabled = true;
    }
  }

  function renderWpList() {
    wpList.innerHTML = '';
    if (!waypoints.length) {
      wpList.innerHTML =
        '<p class="muted pad">该地图暂无航点 · 点「编辑航点」后在地图空白处点击打点，再点「保存航点」</p>';
      updateDeleteGotoButtons();
      return;
    }
    // Drop stale checked indices after list changes
    checkedWpIdx = new Set(
      Array.from(checkedWpIdx).filter((i) => i >= 0 && i < waypoints.length),
    );

    const toolbar = document.createElement('div');
    toolbar.className = 'nav-wp-multi-bar';
    const allChecked =
      waypoints.length > 0 && checkedWpIdx.size === waypoints.length;
    toolbar.innerHTML =
      '<label class="nav-wp-check-all">' +
      `<input type="checkbox" id="wpCheckAll"${allChecked ? ' checked' : ''} />` +
      '<span>全选</span></label>' +
      `<span class="nav-wp-checked-count muted">${
        checkedWpIdx.size ? `已勾选 ${checkedWpIdx.size}` : '勾选后可批量删除'
      }</span>`;
    wpList.appendChild(toolbar);
    const checkAll = toolbar.querySelector('#wpCheckAll');
    if (checkAll) {
      checkAll.onclick = (ev) => ev.stopPropagation();
      checkAll.onchange = () => {
        if (checkAll.checked) {
          checkedWpIdx = new Set(waypoints.map((_, i) => i));
        } else {
          checkedWpIdx = new Set();
        }
        renderWpList();
      };
    }

    waypoints.forEach((wp, idx) => {
      const row = document.createElement('div');
      const bad = !!wp.bad;
      const selected = selectedWpIdx === idx || selectedWp === wp.name;
      const checked = checkedWpIdx.has(idx);
      const charger = isChargerWaypoint(wp);
      row.className =
        'nav-wp-item' +
        (selected ? ' selected' : '') +
        (checked ? ' checked' : '') +
        (bad ? ' wp-bad' : '') +
        (charger ? ' wp-charger' : '');
      row.dataset.wpIdx = String(idx);
      row.innerHTML = `
        <label class="nav-wp-check" title="多选">
          <input type="checkbox" class="nav-wp-check-input"${checked ? ' checked' : ''} />
        </label>
        <div class="nav-wp-item-main">
          <span class="nav-wp-badge">${bad ? '!' : charger ? 'C' : idx + 1}</span>
          <div class="nav-wp-item-text">
            <div class="nav-wp-name">
              <span class="nav-wp-name-label" title="${charger ? '充电桩名称固定' : '双击重命名'}">${escapeHtml(wp.name)}</span>${
                charger ? '<span class="nav-wp-chip">充电桩</span>' : ''
              }${bad ? '<span class="nav-wp-chip bad">坏点</span>' : ''}
            </div>
            <div class="nav-wp-meta mono">x=${wp.x.toFixed(2)} · y=${wp.y.toFixed(2)} · yaw=${wp.yaw.toFixed(2)}</div>
          </div>
        </div>
        ${
          charger
            ? ''
            : '<button type="button" class="nav-wp-rename-btn" title="重命名">重命名</button>'
        }`;
      const checkInput = row.querySelector('.nav-wp-check-input');
      if (checkInput) {
        checkInput.onclick = (ev) => ev.stopPropagation();
        checkInput.onchange = (ev) => {
          ev.stopPropagation();
          if (checkInput.checked) checkedWpIdx.add(idx);
          else checkedWpIdx.delete(idx);
          updateDeleteGotoButtons();
          row.classList.toggle('checked', checkInput.checked);
          const countEl = wpList.querySelector('.nav-wp-checked-count');
          if (countEl) {
            countEl.textContent = checkedWpIdx.size
              ? `已勾选 ${checkedWpIdx.size}`
              : '勾选后可批量删除';
          }
          const allEl = wpList.querySelector('#wpCheckAll');
          if (allEl) allEl.checked = checkedWpIdx.size === waypoints.length;
        };
      }
      row.onclick = () => {
        selectWaypoint(idx, false);
        if (navActive && !bad) goToWaypoint(idx);
      };
      const label = row.querySelector('.nav-wp-name-label');
      if (label && !charger) {
        label.ondblclick = (ev) => {
          ev.preventDefault();
          ev.stopPropagation();
          beginWaypointRename(idx);
        };
      }
      const renameBtn = row.querySelector('.nav-wp-rename-btn');
      if (renameBtn) {
        renameBtn.onclick = (ev) => {
          ev.preventDefault();
          ev.stopPropagation();
          beginWaypointRename(idx);
        };
      }
      wpList.appendChild(row);
    });
    updateDeleteGotoButtons();
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
    if (followEnabled) {
      const fj = await setFollowEnabled(false);
      if (fj.ok) followEnabled = false;
      renderFollowBtn();
      pushLog('>> 前往航点前已关闭人体跟随');
    }
    if (rechargeActive) {
      const rj = await setRechargeEnabled(false);
      if (rj.ok) {
        lastFailHeld = '';
        rechargeActive = false;
        applyRechargeSnapshot({ enabled: false, phase: 'idle', label: '待命' });
      }
      renderRechargeBtn();
      pushLog('>> 前往航点前已停止回充');
    }
    if (window.XwMapCanvas) window.XwMapCanvas.setGoal({ x: wp.x, y: wp.y, yaw: wp.yaw });
    navGoalCycle = 'pending';
    setNavGoalPhase('navigating', wp.name ? `前往 ${wp.name}` : '前往航点');
    const j = await publishGoal(wp.x, wp.y, wp.yaw || 0, 'map');
    if (!j.ok) {
      navGoalCycle = 'idle';
      setNavGoalPhase('failed', j.message || '前往失败');
    }
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
      }
      renderWpList();
    }
    if (orientationControls) orientationControls.syncFromYaw();
  }

  // ——— wire UI ———
  $('refreshMaps').onclick = () => refreshMaps();
  $('loadMapPreview').onclick = () => loadMapPreview();
  $('reloadWp').onclick = () => loadWaypoints();
  $('mapSelect').onchange = async () => {
    if (currentMapName()) {
      rememberMapName(currentMapName());
      await loadMapPreview({ allowLive: navActive });
    } else {
      await loadWaypoints();
    }
  };

  $('startNav').onclick = async () => {
    const name = currentMapName();
    if (!name) {
      alert('请先选择地图');
      return;
    }
    rememberMapName(name);
    // Load static map FIRST so the canvas isn't blank while Nav2 starts.
    const ok = await loadMapPreview({ allowLive: true });
    if (!ok) {
      pushLog('!! 地图加载失败，仍尝试进入导航');
    }
    pushLog(`>> 进入导航 setMode(2) map=${name}`);
    await setMode(2, { map_name: name });
    syncedActiveMap = name;
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
    if (rechargeActive) {
      const rj = await setRechargeEnabled(false);
      if (rj.ok) {
        lastFailHeld = '';
        rechargeActive = false;
        applyRechargeSnapshot({ enabled: false, phase: 'idle', label: '待命' });
      }
      renderRechargeBtn();
    }
    const loop = !!(patrolLoop && patrolLoop.checked);
    navGoalCycle = 'pending';
    setNavGoalPhase('patrol', loop ? '多点巡航（循环）' : '多点巡航');
    const j = await startPatrol({ map_name: name, loop });
    if (!j.ok) {
      navGoalCycle = 'idle';
      setNavGoalPhase('failed', j.message || '巡航失败');
    }
    pushLog(j.ok ? `<< 多点巡航已启动${loop ? '（循环）' : ''}` : `!! ${j.message || 'failed'}`);
  };

  $('cancelNav').onclick = async () => {
    const j = await cancelNav();
    if (j.ok) {
      navGoalCycle = 'idle';
      setNavGoalPhase('cancelled', '已取消');
    }
    if (window.XwMapCanvas && typeof window.XwMapCanvas.clearPlan === 'function') {
      window.XwMapCanvas.clearPlan();
    }
    pushLog(j.ok ? '<< 导航已取消' : `!! ${j.message || 'failed'}`);
  };

  $('gotoSelectedWp').onclick = () => goToWaypoint(selectedWpIdx);

  $('deleteWp').onclick = () => {
    if (!window.XwMapCanvas) return;
    let indices = Array.from(checkedWpIdx);
    if (!indices.length && selectedWpIdx != null) indices = [selectedWpIdx];
    if (!indices.length) {
      flash('请先勾选或选中要删除的航点', 'err');
      return;
    }
    const n = window.XwMapCanvas.deleteWaypointsByIndices(indices);
    if (!n) return;
    waypoints = window.XwMapCanvas.getWaypoints();
    checkedWpIdx = new Set();
    selectedWpIdx = window.XwMapCanvas.getSelectedWaypointIndex();
    selectedWp =
      selectedWpIdx != null && waypoints[selectedWpIdx]
        ? waypoints[selectedWpIdx].name
        : null;
    wpCount.textContent = String(waypoints.length);
    renderWpList();
    pushLog(
      `<< 已删除 ${n} 个航点（未写入文件，请点「保存航点」）`,
    );
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

  function setTaskHint(msg) {
    if (navTaskHint) navTaskHint.textContent = msg || '';
  }

  function renderFollowBtn() {
    if (!navFollowBtn) return;
    navFollowBtn.textContent = followEnabled ? '人体跟随 · 开' : '人体跟随';
    navFollowBtn.className = followEnabled ? 'nav-task-btn is-on' : 'secondary nav-task-btn';
    navFollowBtn.disabled = followBusy || !navActive;
  }

  function renderRechargeBtn() {
    if (!navRechargeBtn) return;
    if (rechargeActive) {
      navRechargeBtn.textContent =
        lastRechargePhase === 'success' ? '停止回充' : '回充中 · 取消';
    } else {
      navRechargeBtn.textContent = '自动回充';
    }
    navRechargeBtn.className = rechargeActive ? 'nav-task-btn is-on' : 'secondary nav-task-btn';
    navRechargeBtn.disabled = rechargeBusy || !navActive;
  }

  function applyRechargeSnapshot(rc) {
    if (!rc || typeof rc !== 'object') return;
    const phase = String(rc.phase || 'idle');
    lastRechargePhase = phase;
    const label = String(rc.label || phase);
    const msg = String(rc.message || '');
    const enabled = !!rc.enabled || !!rc.active;
    if (phase === 'fail') lastFailHeld = msg || label;
    if (phase === 'idle' && enabled === false && !lastFailHeld) lastFailHeld = '';
    if (phase === 'success' || (enabled && phase !== 'fail')) lastFailHeld = '';
    rechargeActive = enabled && phase !== 'fail';
    if (navRechargeStrip) {
      navRechargeStrip.dataset.phase = lastFailHeld && !rechargeActive ? 'fail' : phase;
      const title = lastFailHeld && !rechargeActive ? lastFailHeld : msg || label;
      navRechargeStrip.title = title;
    }
    if (navRechargePhase) {
      navRechargePhase.textContent =
        lastFailHeld && !rechargeActive ? '失败' : label;
    }
    if (navRechargeMsg) {
      navRechargeMsg.textContent =
        lastFailHeld && !rechargeActive ? lastFailHeld : msg && msg !== label ? msg : '';
    }
    const stg = rc.staging;
    if (window.XwMapCanvas && window.XwMapCanvas.setStaging) {
      if (stg && typeof stg.x === 'number') {
        window.XwMapCanvas.setStaging(stg);
      } else if (!rechargeActive) {
        window.XwMapCanvas.setStaging(null);
      }
    }
    renderRechargeBtn();
  }

  async function refreshFollowStatus() {
    try {
      const s = await fetchFollowStatus();
      followEnabled = !!s.enabled;
      renderFollowBtn();
    } catch (_) {
      /* keep last */
    }
  }

  async function refreshRechargeStatus() {
    try {
      applyRechargeSnapshot(await fetchRechargeStatus());
    } catch (_) {
      /* keep last */
    }
  }

  async function toggleFollow() {
    if (followBusy) return;
    if (!navActive && !followEnabled) {
      flash('请先进入导航后再开人体跟随', 'err');
      return;
    }
    followBusy = true;
    renderFollowBtn();
    try {
      const next = !followEnabled;
      if (next && rechargeActive) {
        await setRechargeEnabled(false);
        rechargeActive = false;
        renderRechargeBtn();
      }
      const j = await setFollowEnabled(next);
      if (j.ok) {
        followEnabled = !!j.enabled;
        setTaskHint(
          followEnabled
            ? '跟随已开：点位/巡航已取消，Nav2 仍在运行'
            : '跟随已关：可继续下发点位',
        );
        flash(followEnabled ? '人体跟随已开启' : '人体跟随已关闭', 'ok');
        pushLog(followEnabled ? '>> 人体跟随 ON' : '>> 人体跟随 OFF');
      } else {
        setTaskHint(j.message || '跟随切换失败');
        flash(j.message || '跟随切换失败', 'err');
      }
    } finally {
      followBusy = false;
      await refreshFollowStatus();
    }
  }

  function findChargerIndex() {
    return waypoints.findIndex((w) => isChargerWaypoint(w));
  }

  async function toggleRecharge() {
    if (rechargeBusy) return;
    if (!navActive) {
      flash('请先进入导航后再回充', 'err');
      return;
    }
    rechargeBusy = true;
    renderRechargeBtn();
    try {
      if (rechargeActive) {
        const j = await setRechargeEnabled(false);
        if (j.ok) {
          lastFailHeld = '';
          applyRechargeSnapshot({ ...j, enabled: false, phase: j.phase || 'idle', label: '待命' });
          rechargeActive = false;
          renderRechargeBtn();
          flash('已停止回充', 'ok');
          pushLog('>> 停止自动回充');
        } else {
          flash(j.message || '停止回充失败', 'err');
        }
        return;
      }
      const idx = findChargerIndex();
      if (idx < 0) {
        flash('当前地图没有 charger 充电桩航点', 'err');
        return;
      }
      const wp = waypoints[idx];
      if (wp.bad) {
        flash('充电桩为坏点（距障碍过近），无法回充', 'err');
        return;
      }
      lastFailHeld = '';
      const j = await setRechargeEnabled(true);
      if (j.ok) {
        applyRechargeSnapshot(j);
        flash('已开始自动回充', 'ok');
        pushLog('>> 自动回充');
      } else {
        flash(j.message || '回充启动失败', 'err');
      }
    } finally {
      rechargeBusy = false;
      renderRechargeBtn();
    }
  }

  followTimer = setInterval(refreshFollowStatus, 4000);
  if (navFollowBtn) {
    navFollowBtn.onclick = () => toggleFollow();
    renderFollowBtn();
    refreshFollowStatus();
  }
  if (navRechargeBtn) {
    navRechargeBtn.onclick = () => toggleRecharge();
    renderRechargeBtn();
    refreshRechargeStatus();
  }

  stateHandlerB = (s) => {
    const mode = Number(s.mode);
    const wasNav = navActive;
    navActive = mode === 2 || mode === 3;
    if (!navActive && wasNav) {
      followEnabled = false;
    }
    if (mode === 3) followEnabled = true;
    if (typeof s.detail === 'string' && s.detail.includes('follow=off')) {
      followEnabled = false;
    }
    if (typeof s.detail === 'string' && s.detail.includes('follow=on')) {
      followEnabled = true;
    }
    if (s.recharge) applyRechargeSnapshot(s.recharge);
    applyLocSnapshot(s);
    syncNavGoalFromState(s);
    renderFollowBtn();
    renderRechargeBtn();
  };

  onTask(taskHandler);
  onState(stateHandlerA);
  onState(stateHandlerB);

  if (queryMap && mapSelect) {
    mapSelect.value = queryMap;
    loadMapPreview({ allowLive: navActive });
  }

  return () => {
    if (poseTimer) { clearInterval(poseTimer); poseTimer = null; }
    if (followTimer) { clearInterval(followTimer); followTimer = null; }
    if (taskHandler) { offTask(taskHandler); taskHandler = null; }
    if (stateHandlerA) { offState(stateHandlerA); stateHandlerA = null; }
    if (stateHandlerB) { offState(stateHandlerB); stateHandlerB = null; }
    if (orientationControls?.destroy) { orientationControls.destroy(); orientationControls = null; }
    if (window.XwMapCanvas?.stop) window.XwMapCanvas.stop();
  };
}

let dispose = null;

export function mount(ctx = {}) {
  if (dispose) dispose();
  dispose = wireNavigation(ctx);
  return unmount;
}

export function unmount() {
  if (dispose) { dispose(); dispose = null; }
}
