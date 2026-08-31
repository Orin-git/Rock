import { fetchFoxgloveStatus } from '/js/api.js';

const LS_ORG = 'xw_foxglove_org';
const LS_LAYOUT = 'xw_foxglove_layout';
const DEFAULT_ORG = '5144c9b7';
const DEFAULT_LAYOUT = 'lay_0eXiBaencBo8VMDx';

let refreshTimer = null;
let orgPath = null;
let layoutId = null;

function boardHost() {
  return location.hostname || '127.0.0.1';
}

function buildStudioUrl(ws) {
  const org = (orgPath?.value || '').trim().replace(/^\/+|\/+$/g, '');
  const lay = (layoutId?.value || '').trim();
  let base;
  if (org) base = `https://app.foxglove.dev/${org}/view`;
  else base = 'https://app.foxglove.dev/';
  const u = new URL(base);
  u.searchParams.set('ds', 'foxglove-websocket');
  u.searchParams.set('ds.url', ws);
  if (lay) u.searchParams.set('layoutId', lay);
  return u.toString();
}

function syncLink() {
  const wsUrl = document.getElementById('wsUrl');
  const studioUrlEl = document.getElementById('studioUrl');
  const wsHint = document.getElementById('wsHint');
  const ws = wsUrl?.value || `ws://${boardHost()}:8765`;
  if (studioUrlEl) studioUrlEl.value = buildStudioUrl(ws);
  if (wsHint) wsHint.textContent = ws;
}

async function refresh() {
  const foxMeta = document.getElementById('foxMeta');
  const foxDetail = document.getElementById('foxDetail');
  const wsUrl = document.getElementById('wsUrl');
  const j = await fetchFoxgloveStatus();
  const port = j.port || 8765;
  const ws = `ws://${boardHost()}:${port}`;
  if (wsUrl) wsUrl.value = ws;
  syncLink();
  if (foxMeta) {
    if (j.up) {
      foxMeta.textContent = `在线 · 端口 ${port}`;
      foxMeta.style.color = 'var(--ok)';
    } else {
      foxMeta.textContent = `离线 · 端口 ${port}`;
      foxMeta.style.color = 'var(--bad)';
    }
  }
  if (foxDetail) {
    foxDetail.textContent = j.up
      ? '桥已就绪。请点「新标签打开 Studio」，不要期待页内 iframe。'
      : j.error
        ? `探测失败: ${j.error}`
        : 'Bridge 未就绪。需安装 foxglove_bridge 并 use_foxglove:=true。';
  }
  return j;
}

function onInput() {
  syncLink();
}

function onSave() {
  localStorage.setItem(LS_ORG, orgPath.value.trim());
  localStorage.setItem(LS_LAYOUT, layoutId.value.trim());
  syncLink();
  const foxDetail = document.getElementById('foxDetail');
  if (foxDetail) foxDetail.textContent = '组织路径 / 布局已保存在本浏览器';
}

export function mount() {
  orgPath = document.getElementById('orgPath');
  layoutId = document.getElementById('layoutId');
  if (orgPath) orgPath.value = localStorage.getItem(LS_ORG) || DEFAULT_ORG;
  if (layoutId) layoutId.value = localStorage.getItem(LS_LAYOUT) || DEFAULT_LAYOUT;

  document.getElementById('btnRefresh')?.addEventListener('click', refresh);
  document.getElementById('btnCopyWs')?.addEventListener('click', async () => {
    const wsUrl = document.getElementById('wsUrl');
    const foxDetail = document.getElementById('foxDetail');
    try {
      await navigator.clipboard.writeText(wsUrl?.value || '');
      if (foxDetail) foxDetail.textContent = '已复制 WS 地址';
    } catch (_) {
      wsUrl?.select();
    }
  });
  document.getElementById('btnOpen')?.addEventListener('click', () => {
    syncLink();
    const studioUrlEl = document.getElementById('studioUrl');
    window.open(studioUrlEl?.value, '_blank', 'noopener');
  });
  document.getElementById('btnCopyLink')?.addEventListener('click', async () => {
    syncLink();
    const studioUrlEl = document.getElementById('studioUrl');
    const foxDetail = document.getElementById('foxDetail');
    try {
      await navigator.clipboard.writeText(studioUrlEl?.value || '');
      if (foxDetail) foxDetail.textContent = '已复制 Studio 链接';
    } catch (_) {
      studioUrlEl?.select();
    }
  });
  document.getElementById('btnSave')?.addEventListener('click', onSave);
  orgPath?.addEventListener('input', onInput);
  layoutId?.addEventListener('input', onInput);

  refresh();
  refreshTimer = setInterval(refresh, 5000);
  return unmount;
}

export function unmount() {
  if (refreshTimer) {
    clearInterval(refreshTimer);
    refreshTimer = null;
  }
}
