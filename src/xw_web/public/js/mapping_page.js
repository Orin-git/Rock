import {
  setMode,
  mapManage,
  publishTeleop,
  onTask,
  onState,
  offTask,
  offState,
  setExploreEnabled,
} from '/js/api.js';

let taskHandler = null;
let stateHandler = null;
let stickHandlers = null;

export function mount() {
  const mapNameEl = document.getElementById('mapName');
  const mapStatus = document.getElementById('mapStatus');
  const modeHint = document.getElementById('modeHint');
  const autoHint = document.getElementById('autoHint');
  const startAutoBtn = document.getElementById('startAuto');
  const stopAutoBtn = document.getElementById('stopAuto');
  let mappingActive = false;
  let exploreActive = false;
  let savedOnce = false;

  function pad2(n) {
    return String(n).padStart(2, '0');
  }

  function defaultMapName() {
    const d = new Date();
    return (
      d.getFullYear() +
      pad2(d.getMonth() + 1) +
      pad2(d.getDate()) +
      '_' +
      pad2(d.getHours()) +
      pad2(d.getMinutes()) +
      pad2(d.getSeconds())
    );
  }

  function flashStatus(line) {
    if (mapStatus) mapStatus.textContent = String(line || '').replace(/^(>>|!!|<<)\s*/, '');
  }

  function setExploreUi(on, message) {
    exploreActive = !!on;
    if (startAutoBtn) startAutoBtn.hidden = exploreActive;
    if (stopAutoBtn) stopAutoBtn.hidden = !exploreActive;
    if (autoHint) autoHint.textContent = message || (exploreActive ? '探索中…' : '待命');
  }

  if (mapNameEl) mapNameEl.value = defaultMapName();

  taskHandler = (l) => flashStatus(l);
  onTask(taskHandler);

  stateHandler = (s) => {
    const name = s.mode_name || String(s.mode);
    if (modeHint) modeHint.textContent = `模式：${name} (${s.mode}) · ${s.detail || ''}`;
    mappingActive = Number(s.mode) === 1;
    const ex = s.explore || {};
    if (typeof ex.enabled === 'boolean' || ex.phase) {
      const on = !!ex.enabled || !!ex.active;
      const phase = ex.phase || '';
      const msg = ex.message || '';
      let line = msg;
      if (on && phase === 'exploring') line = msg || '自主探索中';
      else if (phase === 'starting') line = msg || '启动探索栈…';
      else if (phase === 'success') line = msg || '探索完成';
      else if (phase === 'fail') line = msg || '探索失败';
      else if (!on) line = msg || '待命';
      setExploreUi(on && phase !== 'success' && phase !== 'fail' && phase !== 'idle', line);
      if (phase === 'success') savedOnce = true;
    }
  };
  onState(stateHandler);

  if (window.XwMapCanvas) {
    window.XwMapCanvas.start({
      onStatus: (msg) => {
        if (mapStatus) mapStatus.textContent = msg;
      },
    });
  } else if (mapStatus) {
    mapStatus.textContent = 'map_canvas.js 未加载';
  }

  document.getElementById('startManual')?.addEventListener('click', async () => {
    savedOnce = false;
    if (mapNameEl && !mapNameEl.value.trim()) mapNameEl.value = defaultMapName();
    flashStatus('手推建图 setMode(1)');
    await setMode(1, {});
  });

  startAutoBtn?.addEventListener('click', async () => {
    savedOnce = false;
    let name = mapNameEl?.value.trim() || '';
    if (!name && mapNameEl) {
      name = defaultMapName();
      mapNameEl.value = name;
    }
    flashStatus(`自主建图启动 ${name}…`);
    setExploreUi(true, '启动中…');
    const j = await setExploreEnabled(true, name);
    if (!j || !j.ok) {
      setExploreUi(false, (j && j.message) || '启动失败');
      flashStatus(`自主建图失败 ${(j && j.message) || ''}`);
    }
  });

  stopAutoBtn?.addEventListener('click', async () => {
    const name = mapNameEl?.value.trim() || String(Date.now());
    if (!confirm(`停止自主建图并保存地图「${name}」？`)) return;
    flashStatus('停止自主建图…');
    const save = await mapManage(1, name);
    if (save && save.ok) {
      savedOnce = true;
      flashStatus(`已保存 ${name}`);
    } else {
      flashStatus(`保存失败 ${(save && save.message) || ''}（仍将停止探索）`);
    }
    await setExploreEnabled(false);
    await setMode(0, {});
    setExploreUi(false, '待命');
  });

  document.getElementById('stop')?.addEventListener('click', async () => {
    if (exploreActive) {
      if (!confirm('正在自主建图。结束将停止探索并交由后端处理保存。是否结束？')) return;
      await setExploreEnabled(false);
    } else if (mappingActive && !savedOnce) {
      if (
        !confirm(
          '当前建图会话尚未手动保存。\n结束将交由后端 autosave（若已有栅格）。\n是否结束？',
        )
      ) {
        return;
      }
    }
    flashStatus('结束建图 setMode(0)');
    await setMode(0, {});
    setExploreUi(false, '待命');
  });

  document.getElementById('save')?.addEventListener('click', async () => {
    let name = mapNameEl?.value.trim() || '';
    if (!name && mapNameEl) {
      name = defaultMapName();
      mapNameEl.value = name;
    }
    const list = await mapManage(2);
    const maps = list.map_list || [];
    if (maps.indexOf(name) !== -1) {
      if (!confirm(`地图「${name}」已存在，继续保存将覆盖。是否继续？`)) return;
    }
    flashStatus(`保存地图 ${name}…`);
    const j = await mapManage(1, name);
    if (j && j.ok) {
      savedOnce = true;
      flashStatus(`保存成功 ${j.message || name}`);
    } else {
      flashStatus(`保存失败 ${j && j.message ? j.message : ''}`);
    }
  });

  const stick = document.getElementById('stick');
  const knob = document.getElementById('knob');
  const vel = document.getElementById('vel');
  let active = false;

  function setKnob(nx, ny) {
    const r = 48;
    if (knob) knob.style.transform = `translate(calc(-50% + ${nx * r}px), calc(-50% + ${ny * r}px))`;
    const vx = -ny * 0.35;
    const wz = -nx * 0.8;
    if (vel) vel.textContent = `vx=${vx.toFixed(2)} wz=${wz.toFixed(2)}`;
    publishTeleop(vx, wz);
  }

  function fromEvent(e) {
    if (!stick) return;
    const rect = stick.getBoundingClientRect();
    const t = e.touches ? e.touches[0] : e;
    let dx = (t.clientX - (rect.left + rect.width / 2)) / (rect.width / 2);
    let dy = (t.clientY - (rect.top + rect.height / 2)) / (rect.height / 2);
    const mag = Math.hypot(dx, dy) || 1;
    if (mag > 1) {
      dx /= mag;
      dy /= mag;
    }
    setKnob(dx, dy);
  }

  const onDown = (e) => {
    active = true;
    stick.setPointerCapture(e.pointerId);
    fromEvent(e);
  };
  const onMove = (e) => {
    if (active) fromEvent(e);
  };
  const end = () => {
    active = false;
    setKnob(0, 0);
    publishTeleop(0, 0);
  };

  stick?.addEventListener('pointerdown', onDown);
  stick?.addEventListener('pointermove', onMove);
  stick?.addEventListener('pointerup', end);
  stick?.addEventListener('pointercancel', end);
  document.getElementById('teleopStop')?.addEventListener('click', end);
  stickHandlers = { stick, onDown, onMove, end };

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
  if (window.XwMapCanvas?.stop) window.XwMapCanvas.stop();
  publishTeleop(0, 0);
  if (stickHandlers?.stick) {
    const { stick, onDown, onMove, end } = stickHandlers;
    stick.removeEventListener('pointerdown', onDown);
    stick.removeEventListener('pointermove', onMove);
    stick.removeEventListener('pointerup', end);
    stick.removeEventListener('pointercancel', end);
    stickHandlers = null;
  }
}
