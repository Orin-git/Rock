import { publishTeleop, callMotion, setGestureHttps, gestureHttpsStatus } from '/js/api.js';

let gestureTimer = null;
let stickHandlers = null;

export function mount() {
  const stick = document.getElementById('stick');
  const knob = document.getElementById('knob');
  const vel = document.getElementById('vel');
  const jogBtn = document.getElementById('jog');
  const status = document.getElementById('jogStatus');
  let active = false;
  let jogging = false;

  function setKnob(nx, ny) {
    const r = 70;
    if (knob) knob.style.transform = `translate(calc(-50% + ${nx * r}px), calc(-50% + ${ny * r}px))`;
    const vx = -ny * 0.35;
    const wz = -nx * 0.8;
    if (vel) vel.textContent = `vx=${vx.toFixed(2)} · wz=${wz.toFixed(2)}`;
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
  document.getElementById('stop')?.addEventListener('click', end);

  stickHandlers = { stick, onDown, onMove, end };

  async function runJog(ang, dist, label) {
    if (jogging) return;
    jogging = true;
    if (jogBtn) jogBtn.disabled = true;
    const angEl = document.getElementById('ang');
    const distEl = document.getElementById('dist');
    if (angEl) angEl.value = ang;
    if (distEl) distEl.value = dist;
    const dir = Number(dist) > 0 ? '前进' : Number(dist) < 0 ? '后退' : '转向';
    if (status) status.textContent = `${label || dir} 下发中… ang=${ang} dist=${dist}`;
    const j = await callMotion(ang, dist);
    if (status) {
      status.textContent = j.ok
        ? `ok: ${j.message || 'accepted'}`
        : `fail: ${j.message || 'error'}`;
    }
    if (jogBtn) jogBtn.disabled = false;
    jogging = false;
  }

  jogBtn?.addEventListener('click', () =>
    runJog(
      document.getElementById('ang')?.value,
      document.getElementById('dist')?.value,
      '自定义',
    ),
  );

  document.querySelectorAll('[data-jog]').forEach((btn) => {
    btn.addEventListener('click', () => {
      const [a, d] = btn.getAttribute('data-jog').split(',').map(Number);
      runJog(a, d, btn.textContent.trim());
    });
  });

  document.getElementById('jogStop')?.addEventListener('click', async () => {
    publishTeleop(0, 0);
    const j = await callMotion(0, 0);
    if (status) status.textContent = j.ok ? '已停止点动' : `stop: ${j.message || ''}`;
  });

  const gestureOpen = document.getElementById('gestureOpen');
  const gestureHint = document.getElementById('gestureHint');
  const gesturePill = document.getElementById('gesturePill');
  const gestureUrl = `https://${location.hostname || '127.0.0.1'}:9443/gesture_control.html`;
  if (gestureOpen) gestureOpen.href = gestureUrl;

  function syncGesturePill(on) {
    if (!gesturePill) return;
    gesturePill.textContent = on ? '运行中' : '待命';
    gesturePill.className = on ? 'pill on' : 'pill off';
  }

  async function refreshGestureStatus() {
    const j = await gestureHttpsStatus();
    syncGesturePill(!!j.enabled);
    return j;
  }

  const onGestureClick = async (e) => {
    e.preventDefault();
    const w = window.open('about:blank', 'xw-holo-pilot');
    const prev = gestureOpen?.textContent;
    if (gestureOpen) gestureOpen.textContent = '正在启动 HTTPS…';
    if (gestureHint) gestureHint.textContent = '正在拉起 :9443 …';
    const j = await setGestureHttps(true);
    if (!j.ok) {
      if (w) w.close();
      if (gestureOpen) gestureOpen.textContent = prev;
      if (gestureHint) gestureHint.textContent = `启动失败：${j.message || 'error'}`;
      syncGesturePill(false);
      return;
    }
    syncGesturePill(true);
    if (w) w.location.replace(gestureUrl);
    else window.open(gestureUrl, 'xw-holo-pilot');
    if (gestureOpen) gestureOpen.textContent = prev;
    if (gestureHint) {
      gestureHint.textContent =
        `已启动 · ${gestureUrl} · 首次请信任自签证书 · 关页约 10 秒后自动停`;
    }
  };

  gestureOpen?.addEventListener('click', onGestureClick);
  refreshGestureStatus();
  gestureTimer = setInterval(refreshGestureStatus, 4000);

  return unmount;
}

export function unmount() {
  if (gestureTimer) {
    clearInterval(gestureTimer);
    gestureTimer = null;
  }
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
