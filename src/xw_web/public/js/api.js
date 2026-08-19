/** Xiaowei Gen2 console — real HTTP bridge on robot :9000 */

const stateListeners = [];
const taskListeners = [];
const metaListeners = [];
const obstacleListeners = [];

let connected = false;
let pollTimer = null;
let lastTasks = new Set();

let state = {
  mode: 0,
  mode_name: 'IDLE',
  run_mode: 1, // Gen2 default: developer
  safety_ok: true,
  emergency_stop: false,
  power: { battery_percent: 0 },
  detail: '',
  profile: 'normal',
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
    const text = String(line || '');
    let level = 'info';
    if (/(fail|error|失败|错误|异常|超时|unavailable|拒绝|无法)/i.test(text)) level = 'error';
    else if (/(ok|success|成功|完成|已启动|已停止|ready)/i.test(text)) level = 'success';
    pet.showBubble(text, level);
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
  if (!line || lastTasks.has(line)) return;
  lastTasks.add(line);
  if (lastTasks.size > 100) {
    lastTasks = new Set([...lastTasks].slice(-50));
  }
  taskListeners.forEach((fn) => fn(line));
  notifyDesktopPetTask(line);
}

async function fetchState() {
  try {
    const r = await fetch(`${apiBase()}/api/state`, { cache: 'no-store' });
    if (!r.ok) throw new Error(String(r.status));
    const j = await r.json();
    if (j.state) emitState(j.state);
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

export function onObstacle(fn) {
  obstacleListeners.push(fn);
  fn(obstacle);
}

export function onTask(fn) {
  taskListeners.push(fn);
}

export function onMeta(fn) {
  metaListeners.push(fn);
  fn(meta);
}

export function isConnected() {
  return connected;
}

export function publishTeleop(linearX, angularZ) {
  fetch(`${apiBase()}/api/teleop`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ linear_x: linearX, angular_z: angularZ }),
  }).catch(() => {});
}

/** Fixed-distance jog → /api/motion → /xw/motion/command */
export async function callMotion(angleDeg, distanceM, commandId = '') {
  emitTask(`>> motion ang=${angleDeg} dist=${distanceM}`);
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
    emitTask(`<< motion ${j.ok ? 'ok' : 'fail'}: ${j.message || ''}`);
    return j;
  } catch (_) {
    emitTask('!! motion failed');
    return { ok: false, message: 'request failed' };
  }
}

export async function setMode(mode, payload = {}) {
  emitTask(`>> set_mode ${mode}`);
  try {
    const r = await fetch(`${apiBase()}/api/set_mode`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode, payload, command_id: `ui-${Date.now()}` }),
    });
    const j = await r.json();
    emitTask(`<< ${j.message || j.ok}`);
    await fetchState();
    return j;
  } catch (e) {
    emitTask(`!! set_mode failed`);
    return { ok: false };
  }
}

/** Gen2 run_mode: 0 production / 1 developer (default) → /api/run_mode → /xw/supervisor/set_run_mode */
export async function setRunMode(run_mode) {
  const code = Number(run_mode);
  emitTask(`>> set_run_mode ${code === 0 ? '量产' : '开发者'}`);
  try {
    const r = await fetch(`${apiBase()}/api/run_mode`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ run_mode: code }),
    });
    const j = await r.json();
    emitTask(`<< run_mode ${j.label || j.message || ''}`);
    await fetchState();
    return j;
  } catch (_) {
    emitTask('!! set_run_mode failed');
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
    emitTask(`map op=${operation}: ${j.message || ''}`);
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
    emitTask(`waypoint op=${operation}: ${j.message || ''}`);
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
  emitTask(j.message || `pointcloud ${enabled ? 'on' : 'off'}`);
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
  emitTask(j.message || `fall ${enabled ? 'on' : 'off'}`);
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
  emitTask(j.message || `follow ${enabled ? 'on' : 'off'}`);
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
      emitTask(`goal → (${Number(x).toFixed(2)}, ${Number(y).toFixed(2)}) yaw=${Number(yaw).toFixed(2)}`);
    } else {
      emitTask(`!! goal failed: ${j.message || ''}`);
    }
    return j;
  } catch (_) {
    emitTask('!! goal request failed');
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
        ? `initialpose → (${Number(x).toFixed(2)}, ${Number(y).toFixed(2)})`
        : `!! initialpose failed: ${j.message || ''}`,
    );
    return j;
  } catch (_) {
    emitTask('!! initialpose request failed');
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
    emitTask(j.ok ? `patrol ${action}` : `!! patrol failed: ${j.message || ''}`);
    return j;
  } catch (_) {
    emitTask('!! patrol request failed');
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
    emitTask(j.ok ? 'nav cancel' : `!! cancel failed: ${j.message || ''}`);
    return j;
  } catch (_) {
    emitTask('!! cancel request failed');
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
