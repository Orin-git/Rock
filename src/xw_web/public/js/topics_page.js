import {
  fetchGraph,
  watchTopic,
  unwatchTopic,
  peekTopic,
} from '/js/api.js';

let topics = [];
let nodes = [];
let activeTopic = null;
let peekTimer = null;
let graphTimer = null;
let topicFilter = null;
let xwOnly = null;
let btnUnwatch = null;

function escapeHtml(s) {
  return String(s)
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;');
}
function escapeAttr(s) {
  return escapeHtml(s).replace(/'/g, '&#39;');
}

function setProbeIdle(msg) {
  const probeMeta = document.getElementById('probeMeta');
  const probeData = document.getElementById('probeData');
  activeTopic = null;
  if (btnUnwatch) btnUnwatch.disabled = true;
  if (probeMeta) probeMeta.textContent = msg || '未监听任何话题';
  if (probeData) probeData.textContent = '点击左侧话题开始 echo…';
  document.querySelectorAll('.row-item.active').forEach((el) => el.classList.remove('active'));
}

function renderData(data) {
  const probeData = document.getElementById('probeData');
  if (!probeData || data == null) return;
  if (typeof data === 'string') {
    probeData.textContent = data;
    return;
  }
  try {
    probeData.textContent = JSON.stringify(data, null, 2);
  } catch (_) {
    probeData.textContent = String(data);
  }
}

function renderTopics() {
  const topicList = document.getElementById('topicList');
  const topicCount = document.getElementById('topicCount');
  if (!topicList) return;
  const q = (topicFilter?.value || '').trim().toLowerCase();
  const preferXw = xwOnly?.checked;
  let list = topics.slice();
  if (preferXw) {
    list = [
      ...list.filter((t) => t.name.startsWith('/xw')),
      ...list.filter((t) => !t.name.startsWith('/xw')),
    ];
  }
  if (q) {
    list = list.filter(
      (t) =>
        t.name.toLowerCase().includes(q) ||
        (t.hint || '').toLowerCase().includes(q) ||
        (t.types || []).join(' ').toLowerCase().includes(q),
    );
  }
  if (topicCount) topicCount.textContent = String(topics.length);
  if (!list.length) {
    topicList.innerHTML = '<p class="muted pad">无匹配话题</p>';
    return;
  }
  topicList.innerHTML = list
    .map((t) => {
      const types = (t.types || []).join(', ');
      const heavy = t.heavy ? '<span class="chip heavy">大消息</span>' : '';
      const on = activeTopic === t.name ? ' active' : '';
      return `<button type="button" class="row-item topics-row${on}" data-topic="${escapeAttr(t.name)}" data-type="${escapeAttr((t.types || [])[0] || '')}">
            <div class="row-main">
              <span class="row-name">${escapeHtml(t.name)}</span>
              ${heavy}
            </div>
            <div class="row-type">${escapeHtml(types || '—')}</div>
            <div class="row-hint">${escapeHtml(t.hint || '')}</div>
          </button>`;
    })
    .join('');
  topicList.querySelectorAll('.row-item').forEach((btn) => {
    btn.addEventListener('click', () => {
      startWatch(btn.dataset.topic, btn.dataset.type);
    });
  });
}

function renderNodes() {
  const nodeList = document.getElementById('nodeList');
  const nodeCount = document.getElementById('nodeCount');
  if (!nodeList) return;
  if (nodeCount) nodeCount.textContent = String(nodes.length);
  if (!nodes.length) {
    nodeList.innerHTML = '<p class="muted pad">暂无节点（Domain 或服务未起）</p>';
    return;
  }
  nodeList.innerHTML = nodes
    .map(
      (n) => `<div class="row-item static topics-row">
            <div class="row-main"><span class="row-name">${escapeHtml(n.name)}</span></div>
            <div class="row-hint">${escapeHtml(n.hint || '')}</div>
          </div>`,
    )
    .join('');
}

async function loadGraph(force = false) {
  const topicList = document.getElementById('topicList');
  const j = await fetchGraph(force);
  if (!j.ok) {
    if (topicList) {
      topicList.innerHTML = `<p class="muted pad">图不可用：${escapeHtml(j.message || '桥离线')}</p>`;
    }
    return;
  }
  topics = j.topics || [];
  nodes = j.nodes || [];
  renderTopics();
  renderNodes();
}

async function startWatch(topic, type) {
  stopPeekLoop();
  const j = await watchTopic(topic, type);
  const probeMeta = document.getElementById('probeMeta');
  const probeData = document.getElementById('probeData');
  if (!j.ok) {
    setProbeIdle(`监听失败：${j.message || 'error'}`);
    return;
  }
  activeTopic = j.topic || topic;
  if (btnUnwatch) btnUnwatch.disabled = false;
  if (probeMeta) probeMeta.textContent = `${activeTopic}  ·  ${j.type || type || '—'}`;
  if (probeData) {
    probeData.textContent = j.heavy ? '该话题数据过大，仅显示摘要…' : '等待消息…';
  }
  renderTopics();
  startPeekLoop();
}

async function stopWatch() {
  stopPeekLoop();
  await unwatchTopic();
  setProbeIdle();
  renderTopics();
}

function startPeekLoop() {
  stopPeekLoop();
  const tick = async () => {
    if (!activeTopic) return;
    const j = await peekTopic();
    const probeMeta = document.getElementById('probeMeta');
    const probeData = document.getElementById('probeData');
    if (!j.ok || !j.watching) {
      if (activeTopic) setProbeIdle('监听已结束');
      return;
    }
    if (probeMeta) probeMeta.textContent = `${j.watching}  ·  ${j.type || ''}`;
    if (probeData) {
      if (j.error) probeData.textContent = String(j.error);
      else if (j.waiting) probeData.textContent = '已订阅，等待首帧…';
      else if (j.data != null) renderData(j.data);
    }
  };
  tick();
  peekTimer = setInterval(tick, 500);
}

function stopPeekLoop() {
  if (peekTimer) {
    clearInterval(peekTimer);
    peekTimer = null;
  }
}

function onVisibilityChange() {
  if (document.hidden) {
    stopPeekLoop();
    unwatchTopic().then(() => setProbeIdle('页面隐藏，已自动停止监听'));
  }
}

export function mount() {
  topicFilter = document.getElementById('topicFilter');
  xwOnly = document.getElementById('xwOnly');
  btnUnwatch = document.getElementById('btnUnwatch');
  const btnRefresh = document.getElementById('btnRefresh');

  topicFilter?.addEventListener('input', renderTopics);
  xwOnly?.addEventListener('change', renderTopics);
  btnRefresh?.addEventListener('click', () => loadGraph(true));
  btnUnwatch?.addEventListener('click', stopWatch);
  document.addEventListener('visibilitychange', onVisibilityChange);

  loadGraph(true);
  graphTimer = setInterval(() => loadGraph(false), 4000);

  return unmount;
}

export function unmount() {
  stopPeekLoop();
  if (graphTimer) {
    clearInterval(graphTimer);
    graphTimer = null;
  }
  document.removeEventListener('visibilitychange', onVisibilityChange);
  unwatchTopic();
  topics = [];
  nodes = [];
  activeTopic = null;
}
