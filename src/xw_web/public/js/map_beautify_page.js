/**
 * Simplified map editor — draw / erase / one-click beautify / save.
 * Gen2 tradeoffs vs gen1: no keepout layer, no shape select/move, no Foxglove.
 */

import { connect, mapManage } from '/js/api.js';
import '/js/app.js';
import {
  OCCUPIED,
  FREE,
  rasterizeThickLine,
  stampBrush,
} from '/js/map_beautify_algo.js';

connect();

const $ = (id) => document.getElementById(id);

const mapSelect = $('mapSelect');
const statusEl = $('editStatus');
const hintEl = $('coordInfo');
const canvas = $('map-canvas');
const overlay = $('overlay-canvas');
const container = $('map-container');
const brushSizeEl = $('brushSize');
const brushLabel = $('brushSizeLabel');
const toastEl = $('toast');

const ctx = canvas.getContext('2d', { alpha: false });
const octx = overlay.getContext('2d');

let mapName = '';
let width = 0;
let height = 0;
let resolution = 0.05;
let mapData = null; // Uint8Array
let originalData = null;
let dirty = false;
let mode = 'pan'; // pan | draw | erase
let brush = 1;
let undoStack = [];
const MAX_UNDO = 30;

let scale = 1;
let offsetX = 0;
let offsetY = 0;
let bitmap = null;
let bitmapDirty = true;

let dragging = false;
let lastPx = null;
let panStart = null;
let spacePan = false;

function toast(msg, ok = true) {
  if (!toastEl) return;
  toastEl.textContent = msg;
  toastEl.className = 'beautify-toast ' + (ok ? 'ok' : 'err');
  clearTimeout(toast._t);
  toast._t = setTimeout(() => {
    toastEl.textContent = '';
    toastEl.className = 'beautify-toast';
  }, 3200);
}

function setStatus(text) {
  if (statusEl) statusEl.textContent = text;
}

function updateDirtyUi() {
  setStatus(
    mapName
      ? `${mapName} · ${width}×${height} · ${dirty ? '未保存' : '已同步'}`
      : '未加载地图'
  );
  document.querySelectorAll('[data-need-map]').forEach((el) => {
    el.disabled = !mapData;
  });
}

function b64ToBytes(b64) {
  const bin = atob(b64);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}

function bytesToB64(bytes) {
  const chunk = 0x8000;
  let s = '';
  for (let i = 0; i < bytes.length; i += chunk) {
    s += String.fromCharCode.apply(null, bytes.subarray(i, i + chunk));
  }
  return btoa(s);
}

function pushUndo() {
  if (!mapData) return;
  undoStack.push(mapData.slice());
  if (undoStack.length > MAX_UNDO) undoStack.shift();
}

function pgmToRgba(v) {
  if (v <= 50) return [30, 34, 40, 255]; // occupied
  if (v >= 250) return [245, 247, 250, 255]; // free
  return [120, 128, 140, 255]; // unknown
}

function rebuildBitmap() {
  if (!mapData || !width || !height) {
    bitmap = null;
    return;
  }
  const img = ctx.createImageData(width, height);
  const d = img.data;
  for (let i = 0, p = 0; i < mapData.length; i++, p += 4) {
    const c = pgmToRgba(mapData[i]);
    d[p] = c[0];
    d[p + 1] = c[1];
    d[p + 2] = c[2];
    d[p + 3] = c[3];
  }
  if (!bitmap || bitmap.width !== width || bitmap.height !== height) {
    bitmap = document.createElement('canvas');
    bitmap.width = width;
    bitmap.height = height;
  }
  bitmap.getContext('2d').putImageData(img, 0, 0);
  bitmapDirty = false;
}

function resizeCanvases() {
  const r = container.getBoundingClientRect();
  const w = Math.max(1, Math.floor(r.width));
  const h = Math.max(1, Math.floor(r.height));
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
    overlay.width = w;
    overlay.height = h;
  }
}

function fitView() {
  if (!width || !height) return;
  resizeCanvases();
  const pad = 24;
  const sx = (canvas.width - pad * 2) / width;
  const sy = (canvas.height - pad * 2) / height;
  scale = Math.max(0.05, Math.min(sx, sy));
  offsetX = (canvas.width - width * scale) / 2;
  offsetY = (canvas.height - height * scale) / 2;
  draw();
}

function draw() {
  resizeCanvases();
  ctx.fillStyle = '#121820';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  if (!mapData) return;
  if (bitmapDirty || !bitmap) rebuildBitmap();
  ctx.imageSmoothingEnabled = scale < 2;
  ctx.save();
  ctx.translate(offsetX, offsetY);
  ctx.scale(scale, scale);
  ctx.drawImage(bitmap, 0, 0);
  ctx.restore();
  octx.clearRect(0, 0, overlay.width, overlay.height);
}

function screenToMap(sx, sy) {
  const x = Math.floor((sx - offsetX) / scale);
  const y = Math.floor((sy - offsetY) / scale);
  return { x, y };
}

function pointerPos(ev) {
  const r = overlay.getBoundingClientRect();
  return { x: ev.clientX - r.left, y: ev.clientY - r.top };
}

function setMode(next) {
  mode = next;
  document.querySelectorAll('[data-mode]').forEach((btn) => {
    btn.classList.toggle('active', btn.getAttribute('data-mode') === next);
  });
  overlay.style.cursor =
    next === 'pan' || spacePan ? 'grab' : next === 'erase' ? 'cell' : 'crosshair';
}

async function refreshList() {
  const j = await mapManage(2);
  const list = j.map_list || [];
  const prev = mapSelect.value;
  mapSelect.innerHTML = '<option value="">— 选择地图 —</option>';
  list.forEach((name) => {
    const opt = document.createElement('option');
    opt.value = name;
    opt.textContent = name;
    mapSelect.appendChild(opt);
  });
  if (prev && list.includes(prev)) mapSelect.value = prev;
  else if (mapName && list.includes(mapName)) mapSelect.value = mapName;
  if (!j.ok && !list.length) toast(j.message || '列表失败', false);
}

async function loadMap(name) {
  if (!name) {
    toast('请先选择地图', false);
    return;
  }
  toast('正在加载…');
  const j = await mapManage(5, name);
  if (!j.ok) {
    toast(j.message || '加载失败', false);
    return;
  }
  let payload;
  try {
    payload = typeof j.data_json === 'string' ? JSON.parse(j.data_json || '{}') : j.data_json;
  } catch (_) {
    toast('地图数据无效', false);
    return;
  }
  if (!payload || !payload.pgm_b64 || !payload.width || !payload.height) {
    toast('缺少 PGM 像素数据', false);
    return;
  }
  mapName = name;
  width = payload.width | 0;
  height = payload.height | 0;
  resolution = Number(payload.resolution) || 0.05;
  mapData = b64ToBytes(payload.pgm_b64);
  if (mapData.length !== width * height) {
    toast(`像素数量不匹配 ${mapData.length} ≠ ${width * height}`, false);
    mapData = null;
    return;
  }
  originalData = mapData.slice();
  undoStack = [];
  dirty = false;
  bitmapDirty = true;
  mapSelect.value = name;
  fitView();
  updateDirtyUi();
  toast(`已加载 ${name}`, true);
}

function brushRadius() {
  // UI size 1 → single pixel (radius 0); size N → diameter ≈ N
  return Math.max(0, (brush | 0) - 1);
}

function applyPaint(x0, y0, x1, y1) {
  if (!mapData) return;
  const value = mode === 'erase' ? FREE : OCCUPIED;
  const r = brushRadius();
  if (x0 === x1 && y0 === y1) {
    stampBrush(mapData, width, height, x0, y0, value, r);
  } else {
    rasterizeThickLine(mapData, width, height, x0, y0, x1, y1, value, r);
  }
  dirty = true;
  bitmapDirty = true;
  updateDirtyUi();
  draw();
}

function onPointerDown(ev) {
  if (!mapData) return;
  overlay.setPointerCapture(ev.pointerId);
  const p = pointerPos(ev);
  const pan = mode === 'pan' || spacePan || ev.button === 1 || ev.shiftKey;
  dragging = true;
  if (pan) {
    panStart = { x: p.x, y: p.y, ox: offsetX, oy: offsetY };
    overlay.style.cursor = 'grabbing';
    lastPx = null;
    return;
  }
  if (mode === 'draw' || mode === 'erase') {
    pushUndo();
    const m = screenToMap(p.x, p.y);
    lastPx = m;
    applyPaint(m.x, m.y, m.x, m.y);
  }
}

function onPointerMove(ev) {
  if (!mapData) return;
  const p = pointerPos(ev);
  const m = screenToMap(p.x, p.y);
  if (hintEl) {
    hintEl.textContent =
      m.x >= 0 && m.y >= 0 && m.x < width && m.y < height
        ? `像素 (${m.x}, ${m.y}) · 滚轮缩放 · 空格/Shift 拖曳平移`
        : '滚轮缩放 · 空格/Shift 拖曳平移 · 画墙直接写进主地图';
  }
  if (!dragging) return;
  if (panStart) {
    offsetX = panStart.ox + (p.x - panStart.x);
    offsetY = panStart.oy + (p.y - panStart.y);
    draw();
    return;
  }
  if ((mode === 'draw' || mode === 'erase') && lastPx) {
    applyPaint(lastPx.x, lastPx.y, m.x, m.y);
    lastPx = m;
  }
}

function onPointerUp(ev) {
  dragging = false;
  panStart = null;
  lastPx = null;
  try {
    overlay.releasePointerCapture(ev.pointerId);
  } catch (_) {}
  setMode(mode);
}

function onWheel(ev) {
  if (!mapData) return;
  ev.preventDefault();
  const p = pointerPos(ev);
  const before = screenToMap(p.x, p.y);
  const factor = ev.deltaY > 0 ? 0.9 : 1.1;
  scale = Math.min(40, Math.max(0.05, scale * factor));
  offsetX = p.x - before.x * scale;
  offsetY = p.y - before.y * scale;
  draw();
}

function undo() {
  if (!undoStack.length || !mapData) return;
  mapData = undoStack.pop();
  dirty = true;
  bitmapDirty = true;
  updateDirtyUi();
  draw();
}

function resetEdits() {
  if (!originalData) return;
  if (dirty && !confirm('放弃所有未保存修改？')) return;
  pushUndo();
  mapData = originalData.slice();
  dirty = false;
  bitmapDirty = true;
  updateDirtyUi();
  draw();
  toast('已恢复到加载时状态');
}

async function saveMap(asNew) {
  if (!mapData || !mapName) {
    toast('没有可保存的地图', false);
    return;
  }
  let target = mapName;
  if (asNew) {
    const name = prompt('另存为地图名称：', `${mapName}_edit`);
    if (!name || !name.trim()) return;
    target = name.trim();
  } else if (!confirm('保存将自动备份原 PGM。确定覆盖？')) {
    return;
  }

  const payload = JSON.stringify({ pgm_b64: bytesToB64(mapData) });
  toast('正在保存…');
  const j = await mapManage(6, mapName, {
    new_name: asNew ? target : '',
    data_json: payload,
  });
  if (!j.ok) {
    toast(j.message || '保存失败', false);
    return;
  }
  mapName = target;
  originalData = mapData.slice();
  undoStack = [];
  dirty = false;
  mapSelect.value = target;
  await refreshList();
  updateDirtyUi();
  toast(j.message || '已保存');
}

// Wire UI
document.querySelectorAll('[data-mode]').forEach((btn) => {
  btn.addEventListener('click', () => setMode(btn.getAttribute('data-mode')));
});
brushSizeEl.addEventListener('input', () => {
  brush = Number(brushSizeEl.value) || 1;
  brushLabel.textContent = String(brush);
});
$('btnRefreshList').onclick = () => refreshList();
$('btnLoadMap').onclick = () => loadMap(mapSelect.value);
$('btnFitView').onclick = () => fitView();
$('btnUndo').onclick = () => undo();
$('btnReset').onclick = () => resetEdits();
$('btnSave').onclick = () => saveMap(false);
$('btnSaveAs').onclick = () => saveMap(true);

overlay.addEventListener('pointerdown', onPointerDown);
overlay.addEventListener('pointermove', onPointerMove);
overlay.addEventListener('pointerup', onPointerUp);
overlay.addEventListener('pointercancel', onPointerUp);
overlay.addEventListener('wheel', onWheel, { passive: false });
window.addEventListener('resize', () => {
  draw();
});

window.addEventListener('keydown', (ev) => {
  if (ev.code === 'Space' && !ev.repeat) {
    spacePan = true;
    if (!dragging) overlay.style.cursor = 'grab';
    ev.preventDefault();
  }
  if ((ev.ctrlKey || ev.metaKey) && ev.key.toLowerCase() === 'z') {
    ev.preventDefault();
    undo();
  }
});
window.addEventListener('keyup', (ev) => {
  if (ev.code === 'Space') {
    spacePan = false;
    setMode(mode);
  }
});

setMode('pan');
brush = Number(brushSizeEl.value) || 1;
brushLabel.textContent = String(brush);
updateDirtyUi();

(async () => {
  await refreshList();
  const q = new URLSearchParams(location.search).get('map');
  if (q) {
    mapSelect.value = q;
    await loadMap(q);
  }
})();
