/** Xiaowei Gen2 console — real HTTP bridge on robot :9000 */

const stateListeners = [];
const taskListeners = [];
const metaListeners = [];
const obstacleListeners = [];

let connected = false;
let pollTimer = null;
let lastTasks = new Set();

const MODE_ZH = {
  0: '空闲',
  1: '建图',
  2: '导航',
  3: '跟随',
  4: '跌倒监测',
  IDLE: '空闲',
  MAPPING: '建图',
  NAVIGATING: '导航',
  FOLLOWING: '跟随',
  FALL_DETECT: '跌倒监测',
};

const MSG_ZH = {
  ok: '好了',
  done: '好了',
  started: '开始走动',
  driving: '走动中',
  preempted: '换了新动作',
  noop: '不用动',
  busy: '正忙着',
  'recharge on': '开始回充',
  'recharge off': '回充已停',
  'follow on': '开始跟着你',
  'follow off': '不跟了',
  'follow started': '开始跟着你',
  'follow stopped': '不跟了',
  'fall=on': '跌倒监测开了',
  'fall=off': '跌倒监测关了',
  'nav started': '开始导航',
  'nav stopped': '导航停了',
  production: '量产',
  developer: '开发者',
  'cannot recharge while mapping': '建图中不能回充',
  'cannot follow while mapping': '建图中不能跟随',
  'explore on': '开始自主建图',
  'explore off': '自主建图已停',
  'enter navigation with a map first (set_mode 2)': '请先进入导航再回充',
  'follow requires navigation map (set_mode 2 with map_name first)': '请先进入导航再跟随',
  'motor disabled (MCU Flag_Stop)': '电机已停，动不了',
  'request failed': '没连上',
  'goal succeeded': '到啦',
  'goal failed/aborted': '没走到',
  'patrol complete': '巡航走完了',
  'patrol stopped': '巡航停了',
  'target lost': '找不到人了',
  stopped: '停了',
};

function modeZh(modeOrName) {
  if (modeOrName == null) return '空闲';
  const key = String(modeOrName);
  if (Object.prototype.hasOwnProperty.call(MODE_ZH, key)) return MODE_ZH[key];
  const n = Number(modeOrName);
  if (!Number.isNaN(n) && MODE_ZH[n]) return MODE_ZH[n];
  return key;
}

function fmtMeters(m) {
  const n = Number(m);
  if (Number.isNaN(n)) return '';
  const t = Math.abs(n) >= 1 ? n.toFixed(1) : n.toFixed(2);
  return `${t.replace(/\.0$/, '')}米`;
}

/** Plain Chinese for pet / task log — no opcodes, IDs, or jargon. */
function isCleanChinese(s) {
  if (!s) return false;
  if (/[A-Za-z]{2,}/.test(s)) return false;
  if (/[A-Za-z]+\s*=/.test(s) || /=\s*[A-Za-z0-9]/.test(s)) return false;
  if (/[\[\]]/.test(s)) return false;
  return /[\u4e00-\u9fff]/.test(s);
}

function humanizeTaskLine(text) {
  let raw = String(text || '').replace(/\s+/g, ' ').trim();
  if (!raw) return '';
  raw = raw.replace(/[呢呀哦喵～~]+$/u, '').trim();

  // Prefer pet's hard gate if already loaded
  try {
    const pet = window.__DesktopPet;
    if (pet && typeof pet.toPetSpeech === 'function') {
      const viaPet = pet.toPetSpeech(raw);
      if (viaPet) return viaPet;
    }
  } catch (_) { /* noop */ }

  if (isCleanChinese(raw)) return raw;

  let m = raw.match(/set[_\s-]?mode\s*(?:→|->|:)?\s*([A-Za-z_]+|\d+)/i);
  if (m) {
    const mz = modeZh(m[1]);
    return mz ? `切换到${mz}` : '切换模式了';
  }
  m = raw.match(/\b(IDLE|MAPPING|NAVIGATING|FOLLOWING|FALL_DETECT)\b/i);
  if (m && /active\s*=/i.test(raw)) return `切换到${modeZh(m[1]) || '新状态'}`;

  m = raw.match(/^\[progress\]\s*(\w+)\s+(.+)$/i);
  if (m) return humanizeTaskLine(`${m[1]} ${m[2]}`);
  m = raw.match(/^\[result\]\s*(\w+)\s+code=\d+\s*(.*)$/i);
  if (m) return humanizeTaskLine(m[2] ? `${m[1]} ${m[2]}` : `${m[1]} done`);
  m = raw.match(/^\[(\w+)\]\s*(.*)$/);
  if (m) return humanizeTaskLine(m[2] ? `${m[1]} ${m[2]}` : m[1]);

  m = raw.match(/\(\s*(fwd|back)\s+([\d.]+)\s*m\s*\)/i);
  if (m) {
    const dist = fmtMeters(m[2]);
    return m[1].toLowerCase() === 'back' ? `往后走 ${dist}` : `往前走 ${dist}`;
  }
  if (/^(fwd|back)\b/i.test(raw) || /\b(fwd|back)\b/i.test(raw) && /motion|started|driving|accept/i.test(raw)) {
    return /back/i.test(raw) ? '往后走中' : '往前走中';
  }
  if (/^turn\b|err_deg/i.test(raw)) return '转身中';
  if (/timeout/i.test(raw) && /motion|travel|walk|走/i.test(raw)) return '走动超时了';

  if (MSG_ZH[raw]) return MSG_ZH[raw];
  if (MODE_ZH[raw]) return `切换到${MODE_ZH[raw]}`;
  const low = raw.toLowerCase();
  if (MSG_ZH[low]) return MSG_ZH[low];

  m = raw.match(/^(motion|nav|follow|recharge|slam|fall|patrol|map|waypoint|goal|explore)\s+(\S.*)$/i);
  if (m) {
    const cap = m[1].toLowerCase();
    const phase = m[2].trim();
    if (cap === 'motion') {
      if (/start/i.test(phase)) return '开始走动';
      if (/driv/i.test(phase)) return '走动中';
      if (/done|ok|success/i.test(phase)) return '走到啦';
      return humanizeTaskLine(phase) || '走动中';
    }
    if (cap === 'nav' || cap === 'goal') {
      if (/fail|abort/i.test(phase)) return '没走到';
      if (/success|done|complete/i.test(phase)) return '到啦';
      if (/cancel/i.test(phase)) return '导航取消了';
      return '正在前往';
    }
    if (cap === 'follow') {
      if (/off|stop/i.test(phase)) return '不跟了';
      if (/search/i.test(phase)) return '找你中';
      if (/lost/i.test(phase)) return '找不到人了';
      return '跟着你';
    }
    if (cap === 'recharge') {
      const map = {
        idle: '回充待命', nav: '去充电桩', detect: '找充电桩', align: '对准中',
        flip: '掉头中', commit: '贴桩中', retry: '再试一次', success: '充上电了', fail: '回充失败',
        on: '开始回充', off: '回充停了',
      };
      const key = phase.toLowerCase().split(/\s+/)[0];
      return map[key] || '正在回充';
    }
    if (cap === 'explore') {
      if (/fail/i.test(phase)) return '自主建图失败';
      if (/success|finish|done/i.test(phase)) return '自主建图完成';
      if (/stop|idle|off/i.test(phase)) return '自主建图已停';
      if (/start/i.test(phase)) return '自主建图启动中';
      return '自主建图探索中';
    }
    if (cap === 'patrol') return /stop/i.test(phase) ? '巡航停了' : '开始巡航';
    if (cap === 'fall') return /off/i.test(phase) ? '跌倒监测关了' : '跌倒监测开了';
    if (cap === 'map') return '地图更新了';
    if (cap === 'waypoint') return '航点更新了';
    if (cap === 'slam') return '建图中';
  }

  if (/^fall=/i.test(raw)) return /on/i.test(raw) ? '跌倒监测开了' : '跌倒监测关了';
  if (/busy/i.test(low)) return '正忙着，稍后再试';
  if (/^accepted\b/i.test(raw)) return '收到，开始动';
  if (/initialpose|initial.?pose/i.test(raw)) return '位置定好了';
  if (/fail|error|unavailable|reject|invalid/i.test(low)) return '没成功，再试一次';
  if (/^(ok|done|success|ready)$/i.test(raw)) return '好了';

  // Hard gate: never emit English / key=value to UI or pet
  if (!isCleanChinese(raw)) {
    const stripped = raw
      .replace(/[A-Za-z][A-Za-z0-9_./-]*/g, ' ')
      .replace(/[=\[\]<>(){}|]+/g, ' ')
      .replace(/\s+/g, ' ')
      .trim();
    if (isCleanChinese(stripped) && stripped.length >= 2) return stripped;
    return '';
  }
  return raw;
}

function zhMessage(text) {
  return humanizeTaskLine(text);
}

function describeMotion(angleDeg, distanceM) {
  const ang = Number(angleDeg) || 0;
  const dist = Number(distanceM) || 0;
  const parts = [];
  if (Math.abs(ang) > 0.5) {
    parts.push(ang > 0 ? `左转${Math.abs(ang).toFixed(0)}°` : `右转${Math.abs(ang).toFixed(0)}°`);
  }
  if (Math.abs(dist) > 0.001) {
    parts.push(dist >= 0 ? `往前走 ${fmtMeters(dist)}` : `往后走 ${fmtMeters(Math.abs(dist))}`);
  }
  return parts.length ? parts.join('，') : '原地不动';
}

let state = {
  mode: 0,
  mode_name: 'IDLE',
  run_mode: 1, // Gen2 default: developer
  safety_ok: true,
  emergency_stop: false,
  power: { battery_percent: 0 },
  detail: '',
  profile: 'normal',
  recharge: {
    enabled: false,
    phase: 'idle',
    message: '待命',
    label: '待命',
    result: '',
    retries: 0,
    staging: null,
  },
};

let obstacle = {
  blocked: false,
  any_sector_blocked: false,
  safety_ok: true,
  reason: 'waiting',
  depth_m: null,
  sectors: {
    front: { blocked: false, range_m: null, source: null },
    rear: { blocked: false, range_m: null, source: null },
    left: { blocked: false, range_m: null, source: null },
    right: { blocked: false, range_m: null, source: null },
  },
};

let meta = {
  ros_domain_id: '',
  robot_id: '',
  services: { stack_up: false, supervisor_up: false },
};

function apiBase() {
  return `${window.location.protocol}//${window.location.host}`;
}

function setConn(on, label) {
  connected = on;
  const el = document.getElementById('conn');
  if (!el) return;
  el.className = on ? 'pill on' : 'pill off';
  el.textContent = label || (on ? '链路在线' : '链路离线');
}

function updateDomainUi() {
  const d = meta.ros_domain_id || '—';
  const stack = meta.services?.stack_up;
  const el = document.getElementById('domainBadge');
  if (el) {
    el.textContent = `DOMAIN ${d}`;
    el.className = 'pill domain-badge ' + (stack ? 'on' : connected ? 'warn' : 'off');
    el.title = stack
      ? '机器人服务已就绪（Supervisor / Map）'
      : connected
        ? '网页桥在线，但核心服务未就绪'
        : '服务未启动';
  }
  const svc = document.getElementById('svcBadge');
  if (svc) {
    if (!connected) {
      svc.textContent = '服务离线';
      svc.className = 'pill off';
    } else if (stack) {
      svc.textContent = '服务已启动';
      svc.className = 'pill on';
    } else {
      svc.textContent = '服务启动中…';
      svc.className = 'pill warn';
    }
  }
  metaListeners.forEach((fn) => fn(meta));
}

function notifyDesktopPetState(s) {
  try {
    const pet = window.__DesktopPet;
    if (!pet) return;
    if (typeof pet.setGen2Mode === 'function') {
      pet.setGen2Mode(s.mode, s.mode_name);
    } else if (typeof pet.setTaskMode === 'function') {
      const m = Number(s.mode);
      const mapped = m === 1 ? 2 : m === 2 ? 1 : m === 3 ? 3 : 0;
      pet.setTaskMode(mapped);
    }
    /* Gen2 RobotState.run_mode: 0 production / 1 developer (default) */
    const rm = s.run_mode == null ? 1 : Number(s.run_mode);
    if (typeof pet.setGen2RunMode === 'function') {
      pet.setGen2RunMode(rm);
    } else if (typeof pet.setRunMode === 'function') {
      pet.setRunMode(rm);
    }
  } catch (_) { /* noop */ }
}

function notifyDesktopPetTask(line) {
  try {
    const pet = window.__DesktopPet;
    if (!pet || typeof pet.showBubble !== 'function') return;
    const text = humanizeTaskLine(line);
    if (!text) return;
    let level = 'info';
    if (/(失败|错误|异常|超时|拒绝|无法|丢|没走)/.test(text)) level = 'error';
    else if (/(成功|完成|到啦|好了|开了|走到|充上)/.test(text)) level = 'success';
    /* plain: no「摸鱼中」装饰，只说在干啥 */
    pet.showBubble(text, level, { plain: true });
  } catch (_) { /* noop */ }
}

function emitState(s) {
  state = { ...state, ...s };
  stateListeners.forEach((fn) => fn(state));
  notifyDesktopPetState(state);
}

function emitObstacle(o) {
  if (!o || typeof o !== 'object') return;
  obstacle = {
    ...obstacle,
    ...o,
    sectors: {
      ...obstacle.sectors,
      ...(o.sectors || {}),
    },
  };
  obstacleListeners.forEach((fn) => fn(obstacle));
}

function emitTask(line) {
  const text = humanizeTaskLine(line);
  if (!text || lastTasks.has(text)) return;
  lastTasks.add(text);
  if (lastTasks.size > 100) {
    lastTasks = new Set([...lastTasks].slice(-50));
  }
  taskListeners.forEach((fn) => fn(text));
  notifyDesktopPetTask(text);
}

async function fetchState() {
  try {
    const r = await fetch(`${apiBase()}/api/state`, { cache: 'no-store' });
    if (!r.ok) throw new Error(String(r.status));
    const j = await r.json();
    if (j.state) emitState({
      ...j.state,
      recharge: j.recharge || state.recharge || {},
      explore: j.explore || state.explore || {},
    });
    if (j.obstacle) emitObstacle(j.obstacle);
    if (Array.isArray(j.tasks)) {
      j.tasks.slice().reverse().forEach((t) => emitTask(t));
    }
    meta = {
      ros_domain_id: j.ros_domain_id || '',
      robot_id: j.robot_id || '',
      services: j.services || { stack_up: false },
    };
    const d = meta.ros_domain_id ? `D${meta.ros_domain_id}` : '?';
    const svc = meta.services?.stack_up ? '栈就绪' : '栈未全起';
    setConn(true, `桥在线 · ${d} · ${svc}`);
    updateDomainUi();
    return true;
  } catch (_) {
    setConn(false, '桥未就绪 · 服务未启动');
    meta = { ros_domain_id: '', robot_id: '', services: { stack_up: false } };
    updateDomainUi();
    return false;
  }
}

export function connect() {
  fetchState();
  if (!pollTimer) pollTimer = setInterval(fetchState, 500);
}

export function onState(fn) {
  stateListeners.push(fn);
  fn(state);
}

export function offState(fn) {
  const i = stateListeners.indexOf(fn);
  if (i >= 0) stateListeners.splice(i, 1);
}

export function onObstacle(fn) {
  obstacleListeners.push(fn);
  fn(obstacle);
}

export function offObstacle(fn) {
  const i = obstacleListeners.indexOf(fn);
  if (i >= 0) obstacleListeners.splice(i, 1);
}

export function onTask(fn) {
  taskListeners.push(fn);
}

export function offTask(fn) {
  const i = taskListeners.indexOf(fn);
  if (i >= 0) taskListeners.splice(i, 1);
}

export function onMeta(fn) {
  metaListeners.push(fn);
  fn(meta);
}

export function offMeta(fn) {
  const i = metaListeners.indexOf(fn);
  if (i >= 0) metaListeners.splice(i, 1);
}

export function isConnected() {
  return connected;
}

/** On-demand HTTPS:9443 gesture teleop (idle-exits when unused). */
export async function setGestureHttps(enabled) {
  try {
    const r = await fetch(`${apiBase()}/api/gesture`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enabled: !!enabled }),
    });
    return await r.json();
  } catch (_) {
    return { ok: false, message: 'request failed' };
  }
}

export async function gestureHttpsStatus() {
  try {
    const r = await fetch(`${apiBase()}/api/gesture`, { cache: 'no-store' });
    return await r.json();
  } catch (_) {
    return { ok: false, enabled: false };
  }
}

/**
 * Joystick teleop publish — match gesture_control.js cadence.
 * Arbiter stale_timeout is 0.4s; identical non-zero twists must refresh sooner
 * or the chassis stop/starts (jerky). Coalesce while a POST is in flight.
 */
const TELEOP_PUBLISH_INTERVAL_MS = 100;
const TELEOP_MOVING_HEARTBEAT_MS = 250;
const TELEOP_IDLE_HEARTBEAT_MS = 800;

let teleopDesiredLin = 0;
let teleopDesiredAng = 0;
let teleopLastPostedLin = null;
let teleopLastPostedAng = null;
let teleopLastPostAt = 0;
let teleopInFlight = false;
let teleopFlushQueued = false;
let teleopFlushForce = false;
let teleopTimer = null;

function flushTeleop(force = false) {
  const lin = teleopDesiredLin;
  const ang = teleopDesiredAng;
  const now = performance.now();
  const sameAsLast = teleopLastPostedLin === lin && teleopLastPostedAng === ang;
  const moving = Math.abs(lin) > 1e-4 || Math.abs(ang) > 1e-4;
  const heartbeat = moving ? TELEOP_MOVING_HEARTBEAT_MS : TELEOP_IDLE_HEARTBEAT_MS;
  if (!force && sameAsLast && now - teleopLastPostAt < heartbeat) {
    return;
  }
  if (teleopInFlight) {
    teleopFlushQueued = true;
    teleopFlushForce = teleopFlushForce || force;
    return;
  }

  teleopLastPostedLin = lin;
  teleopLastPostedAng = ang;
  teleopLastPostAt = now;
  teleopInFlight = true;

  fetch(`${apiBase()}/api/teleop`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ linear_x: lin, angular_z: ang }),
    cache: 'no-store',
  })
    .catch(() => {})
    .finally(() => {
      teleopInFlight = false;
      if (teleopFlushQueued) {
        const nextForce = teleopFlushForce;
        teleopFlushQueued = false;
        teleopFlushForce = false;
        flushTeleop(nextForce);
      }
    });
}

function ensureTeleopTimer() {
  if (teleopTimer) return;
  teleopTimer = setInterval(() => flushTeleop(false), TELEOP_PUBLISH_INTERVAL_MS);
}

export function publishTeleop(linearX, angularZ) {
  teleopDesiredLin = Number(linearX) || 0;
  teleopDesiredAng = Number(angularZ) || 0;
  ensureTeleopTimer();
  const isStop = Math.abs(teleopDesiredLin) < 1e-4 && Math.abs(teleopDesiredAng) < 1e-4;
  if (isStop) {
    flushTeleop(true);
  }
}

/** Fixed-distance jog → /api/motion → /xw/motion/command */
export async function callMotion(angleDeg, distanceM, commandId = '') {
  const tip = describeMotion(angleDeg, distanceM);
  emitTask(tip);
  try {
    const r = await fetch(`${apiBase()}/api/motion`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        angle_deg: Number(angleDeg),
        distance_m: Number(distanceM),
        command_id: commandId || `ui-${Date.now()}`,
      }),
    });
    const j = await r.json();
    emitTask(j.ok ? tip : `没动起来 · ${humanizeTaskLine(j.message) || ''}`.trim());
    return j;
  } catch (_) {
    emitTask('走动请求失败');
    return { ok: false, message: 'request failed' };
  }
}

export async function setMode(mode, payload = {}) {
  emitTask(`切换模式 → ${modeZh(mode)}`);
  try {
    const r = await fetch(`${apiBase()}/api/set_mode`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode, payload, command_id: `ui-${Date.now()}` }),
    });
    const j = await r.json();
    const tip = zhMessage(j.message) || modeZh(j.active_mode ?? mode);
    emitTask(j.ok ? `已切换到${tip}` : `模式切换失败 · ${tip}`);
    await fetchState();
    return j;
  } catch (e) {
    emitTask('模式切换失败');
    return { ok: false };
  }
}

/** Gen2 run_mode: 0 production / 1 developer (default) → /api/run_mode → /xw/supervisor/set_run_mode */
export async function setRunMode(run_mode) {
  const code = Number(run_mode);
  emitTask(`切换运行形态 → ${code === 0 ? '量产' : '开发者'}`);
  try {
    const r = await fetch(`${apiBase()}/api/run_mode`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ run_mode: code }),
    });
    const j = await r.json();
    emitTask(j.ok ? `已切换为${j.label || (code === 0 ? '量产' : '开发者')}形态` : '运行形态切换失败');
    await fetchState();
    return j;
  } catch (_) {
    emitTask('运行形态切换失败');
    return { ok: false, run_mode: 1 };
  }
}

export async function mapManage(operation, map_name = '', extra = {}) {
  try {
    const r = await fetch(`${apiBase()}/api/map`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ operation, map_name, ...extra }),
    });
    const j = await r.json();
    emitTask(`地图：${zhMessage(j.message) || (j.ok ? '好了' : '没成功')}`);
    return j;
  } catch (_) {
    return { ok: false, map_list: [] };
  }
}

/** Waypoint / pointList manage → /api/waypoint → /xw/waypoint/manage */
export async function waypointManage(operation, map_name = '', extra = {}) {
  try {
    const r = await fetch(`${apiBase()}/api/waypoint`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ operation, map_name, ...extra }),
    });
    const j = await r.json();
    emitTask(`航点：${zhMessage(j.message) || (j.ok ? '好了' : '没成功')}`);
    return j;
  } catch (_) {
    return { ok: false, names: [], data_json: '' };
  }
}

/** ROS graph metadata only (no message traffic). */
export async function fetchGraph(force = false) {
  try {
    const q = force ? '?force=1' : '';
    const r = await fetch(`${apiBase()}/api/graph${q}`, { cache: 'no-store' });
    if (!r.ok) throw new Error(String(r.status));
    return await r.json();
  } catch (_) {
    return { ok: false, topics: [], nodes: [], message: 'bridge offline' };
  }
}

/** Start single on-demand topic probe (replaces any previous watch). */
export async function watchTopic(topic, type = '') {
  try {
    const r = await fetch(`${apiBase()}/api/topic/watch`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ topic, type, lease_sec: 45 }),
    });
    return await r.json();
  } catch (_) {
    return { ok: false, message: 'watch failed' };
  }
}

export async function unwatchTopic() {
  try {
    const r = await fetch(`${apiBase()}/api/topic/unwatch`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: '{}',
    });
    return await r.json();
  } catch (_) {
    return { ok: false };
  }
}

export async function peekTopic() {
  try {
    const r = await fetch(`${apiBase()}/api/topic/peek`, { cache: 'no-store' });
    if (!r.ok) throw new Error(String(r.status));
    return await r.json();
  } catch (_) {
    return { ok: false, watching: null, data: null };
  }
}

/** Foxglove bridge TCP probe (port 8765). */
export async function fetchFoxgloveStatus() {
  try {
    const r = await fetch(`${apiBase()}/api/foxglove`, { cache: 'no-store' });
    if (!r.ok) throw new Error(String(r.status));
    return await r.json();
  } catch (_) {
    return { ok: false, up: false, port: 8765, error: 'bridge offline' };
  }
}

/** Depth pointcloud debug switch (default off; nav also auto-enables). */
export async function fetchPointcloudStatus() {
  const r = await fetch(`${apiBase()}/api/pointcloud`, { cache: 'no-store' });
  if (!r.ok) throw new Error(String(r.status));
  return await r.json();
}

export async function setPointcloudEnabled(enabled) {
  const r = await fetch(`${apiBase()}/api/pointcloud`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ enabled: !!enabled }),
  });
  const j = await r.json();
  emitTask(zhMessage(j.message) || `点云调试已${enabled ? '开启' : '关闭'}`);
  return j;
}

/** Fall detection orthogonal switch (can run with IDLE/nav). */
export async function fetchFallStatus() {
  const r = await fetch(`${apiBase()}/api/fall`, { cache: 'no-store' });
  if (!r.ok) throw new Error(String(r.status));
  return await r.json();
}

export async function setFallEnabled(enabled) {
  const r = await fetch(`${apiBase()}/api/fall`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ enabled: !!enabled }),
  });
  const j = await r.json();
  emitTask(zhMessage(j.message) || `跌倒监测已${enabled ? '开启' : '关闭'}`);
  return j;
}

/** Body-follow orthogonal task (requires nav; does not tear down Nav2). */
export async function fetchFollowStatus() {
  const r = await fetch(`${apiBase()}/api/follow`, { cache: 'no-store' });
  if (!r.ok) throw new Error(String(r.status));
  return await r.json();
}

export async function setFollowEnabled(enabled) {
  const r = await fetch(`${apiBase()}/api/follow`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ enabled: !!enabled }),
  });
  const j = await r.json();
  emitTask(zhMessage(j.message) || `跟随已${enabled ? '开启' : '关闭'}`);
  return j;
}

/** Auto-recharge orthogonal task (requires nav; Laser-Lock Dock). */
export async function fetchRechargeStatus() {
  const r = await fetch(`${apiBase()}/api/recharge`, { cache: 'no-store' });
  if (!r.ok) throw new Error(String(r.status));
  return await r.json();
}

export async function setRechargeEnabled(enabled) {
  const r = await fetch(`${apiBase()}/api/recharge`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ enabled: !!enabled }),
  });
  const j = await r.json();
  emitTask(zhMessage(j.message) || `回充已${enabled ? '开启' : '关闭'}`);
  return j;
}

/** Autonomous mapping (frontier) — orthogonal on MAPPING mode. */
export async function fetchExploreStatus() {
  const r = await fetch(`${apiBase()}/api/explore`, { cache: 'no-store' });
  if (!r.ok) throw new Error(String(r.status));
  return await r.json();
}

export async function setExploreEnabled(enabled, mapName = '') {
  const body = { enabled: !!enabled };
  if (mapName) body.map_name = String(mapName);
  const r = await fetch(`${apiBase()}/api/explore`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  const j = await r.json();
  emitTask(zhMessage(j.message) || `自主建图已${enabled ? '开启' : '关闭'}`);
  return j;
}

/** Publish nav goal → /api/goal → /xw/goal_pose */
export async function publishGoal(x, y, yaw = 0, frame_id = 'map') {
  try {
    const r = await fetch(`${apiBase()}/api/goal`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ x, y, yaw, frame_id }),
    });
    const j = await r.json();
    if (j.ok) {
      emitTask('去那边');
    } else {
      emitTask(`前往失败 · ${zhMessage(j.message) || '没发出去'}`.replace(/\s·\s$/, ''));
    }
    return j;
  } catch (_) {
    emitTask('前往请求失败');
    return { ok: false };
  }
}

/** AMCL initial pose → /api/initialpose → /initialpose */
export async function publishInitialPose(x, y, yaw = 0, frame_id = 'map') {
  try {
    const r = await fetch(`${apiBase()}/api/initialpose`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ x, y, yaw, frame_id }),
    });
    const j = await r.json();
    emitTask(
      j.ok
        ? '位置定好了'
        : `定位失败 · ${zhMessage(j.message) || '没发出去'}`.replace(/\s·\s$/, ''),
    );
    return j;
  } catch (_) {
    emitTask('初位姿请求失败');
    return { ok: false };
  }
}

/** Multi-point patrol → /api/nav/patrol → /xw/nav/patrol_cmd */
export async function startPatrol({ map_name = '', loop = false, waypoints = null, action = 'start' } = {}) {
  try {
    const body = { map_name, loop: !!loop, action };
    if (Array.isArray(waypoints)) body.waypoints = waypoints;
    const r = await fetch(`${apiBase()}/api/nav/patrol`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    const j = await r.json();
    if (j.ok) {
      emitTask(action === 'stop' ? '巡航已停止' : `巡航已启动${loop ? '（循环）' : ''}`);
    } else {
      emitTask(`巡航失败 · ${zhMessage(j.message) || ''}`);
    }
    return j;
  } catch (_) {
    emitTask('巡航请求失败');
    return { ok: false };
  }
}

/** Cancel current NavigateToPose / patrol */
export async function cancelNav() {
  try {
    const r = await fetch(`${apiBase()}/api/nav/cancel`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: '{}',
    });
    const j = await r.json();
    emitTask(j.ok ? '导航已取消' : `取消失败 · ${zhMessage(j.message) || ''}`);
    return j;
  } catch (_) {
    emitTask('取消请求失败');
    return { ok: false };
  }
}

/** Sensor hub presence (lidar / depth / placeholders). */
export async function fetchSensorHub() {
  try {
    const r = await fetch(`${apiBase()}/api/sensors`, { cache: 'no-store' });
    if (!r.ok) throw new Error(String(r.status));
    return await r.json();
  } catch (_) {
    return { ok: false, sensors: {}, layout: [] };
  }
}
