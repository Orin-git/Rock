/** Xiaowei Gen2 console — real HTTP bridge on robot :9000 */

const stateListeners = [];
const taskListeners = [];

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
    const domain = j.ros_domain_id ? ` · D${j.ros_domain_id}` : '';
    setConn(true, `ROS 桥在线${domain}`);
    return true;
  } catch (_) {
    setConn(false, '桥未就绪 · 请启动 launch');
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
