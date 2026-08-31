import {
  setMode,
  setRunMode,
  onTask,
  onState,
  offTask,
  offState,
  fetchPointcloudStatus,
  setPointcloudEnabled,
  fetchFallStatus,
  setFallEnabled,
} from '/js/api.js';

let taskHandler = null;
let stateHandler = null;
let flashTimer = null;
let toggleCleanups = [];

export function mount() {
  const taskFlash = document.getElementById('taskFlash');
  const defaultLead = taskFlash ? taskFlash.textContent : '';

  function flashTask(line) {
    if (!taskFlash) return;
    taskFlash.textContent = String(line || '').replace(/^(>>|!!|<<)\s*/, '') || defaultLead;
    clearTimeout(flashTimer);
    flashTimer = setTimeout(() => {
      taskFlash.textContent = defaultLead;
    }, 4500);
  }

  taskHandler = (l) => flashTask(l);
  onTask(taskHandler);

  document.getElementById('idle')?.addEventListener('click', () => setMode(0, {}));

  const runPill = document.getElementById('runPill');
  const runHint = document.getElementById('runHint');
  const runKpi = document.getElementById('runKpi');
  const runDev = document.getElementById('runDev');
  const runProd = document.getElementById('runProd');

  function renderRunMode(rm) {
    const code = rm == null ? 1 : Number(rm);
    const isDev = code !== 0;
    if (runPill) {
      runPill.textContent = isDev ? '开发者' : '量产';
      runPill.className = isDev ? 'pill on' : 'pill warn';
    }
    if (runKpi) runKpi.textContent = isDev ? '开发者' : '量产';
    if (runDev) runDev.className = isDev ? '' : 'secondary';
    if (runProd) runProd.className = isDev ? 'secondary' : '';
    if (runHint) runHint.textContent = isDev ? '建图/导航可互切' : '建图与导航互斥';
  }

  stateHandler = (s) => renderRunMode(s.run_mode);
  onState(stateHandler);

  runDev?.addEventListener('click', () => setRunMode(1));
  runProd?.addEventListener('click', () => setRunMode(0));

  function wireToggle({ btn, pill, hint, fetchStatus, setEnabled }) {
    let enabled = false;
    let busy = false;
    let refreshTimer = null;

    function render() {
      btn.textContent = enabled ? '关闭' : '开启';
      btn.className = 'settings-cap-btn ' + (enabled ? '' : 'secondary');
      pill.textContent = enabled ? 'ON' : 'OFF';
      pill.className = enabled ? 'pill on' : 'pill off';
    }

    async function refresh() {
      try {
        const s = await fetchStatus();
        enabled = !!s.enabled;
        hint.textContent = s.service_ready
          ? `${s.hint || ''} · ${s.topic || ''}`
          : s.message || '服务未就绪';
        render();
      } catch (e) {
        hint.textContent = String(e.message || e);
      }
    }

    const onClick = async () => {
      if (busy) return;
      busy = true;
      btn.disabled = true;
      try {
        const j = await setEnabled(!enabled);
        if (j.ok) enabled = !!j.enabled;
        else hint.textContent = j.message || '切换失败';
        render();
        await refresh();
      } finally {
        busy = false;
        btn.disabled = false;
      }
    };

    btn.addEventListener('click', onClick);
    refresh();
    refreshTimer = setInterval(refresh, 4000);
    toggleCleanups.push(() => {
      clearInterval(refreshTimer);
      btn.removeEventListener('click', onClick);
    });
  }

  wireToggle({
    btn: document.getElementById('fallToggle'),
    pill: document.getElementById('fallPill'),
    hint: document.getElementById('fallHint'),
    fetchStatus: fetchFallStatus,
    setEnabled: setFallEnabled,
  });

  wireToggle({
    btn: document.getElementById('pcToggle'),
    pill: document.getElementById('pcPill'),
    hint: document.getElementById('pcHint'),
    fetchStatus: fetchPointcloudStatus,
    setEnabled: setPointcloudEnabled,
  });

  return unmount;
}

export function unmount() {
  if (taskHandler) {
    offTask(taskHandler);
    taskHandler = null;
  }
  if (stateHandler) {
    offState(stateHandler);
    stateHandler = null;
  }
  if (flashTimer) clearTimeout(flashTimer);
  toggleCleanups.forEach((fn) => fn());
  toggleCleanups = [];
}
