/** Xiaowei Gen2 console — real HTTP bridge on robot :9000 */

const stateListeners = [];
const taskListeners = [];
const metaListeners = [];

let connected = false;
let pollTimer = null;
let lastTasks = new Set();

let state = {
  mode: 0,
  mode_name: 'IDLE',
  safety_ok: true,
  emergency_stop: false,
  power: { battery_percent: 0 },
  detail: '',
  profile: 'normal',
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

function emitState(s) {
  state = { ...state, ...s };
  stateListeners.forEach((fn) => fn(state));
}

function emitTask(line) {
  if (!line || lastTasks.has(line)) return;
  lastTasks.add(line);
  if (lastTasks.size > 100) {
    lastTasks = new Set([...lastTasks].slice(-50));
  }
  taskListeners.forEach((fn) => fn(line));
}

async function fetchState() {
  try {
    const r = await fetch(`${apiBase()}/api/state`, { cache: 'no-store' });
    if (!r.ok) throw new Error(String(r.status));
    const j = await r.json();
    if (j.state) emitState(j.state);
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
