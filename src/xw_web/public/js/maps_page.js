import { mapManage, waypointManage, onTask, offTask } from '/js/api.js';
import { navigateTo } from '/js/navigate.js';

let currentMaps = [];
let pointListNames = [];
let pointListMeta = {};
let pointListSet = new Set();
let busy = false;
let wpBusy = false;
let statusTimer = null;
let taskHandler = null;

function setStatus(line, isErr) {
  const statusEl = document.getElementById('mapsStatus');
  if (!statusEl) return;
  statusEl.textContent = line || '';
  statusEl.classList.toggle('is-err', !!isErr);
  if (statusTimer) clearTimeout(statusTimer);
  if (line) {
    statusTimer = setTimeout(() => {
      statusEl.textContent = '';
      statusEl.classList.remove('is-err');
    }, 5000);
  }
}

function pointListNameFor(mapName) {
  if (!mapName) return '';
  return mapName.endsWith('_pointList') ? mapName : `${mapName}_pointList`;
}

function mapNameFromPointList(listName) {
  if (!listName) return '';
  return listName.endsWith('_pointList') ? listName.slice(0, -'_pointList'.length) : listName;
}

function getSelected() {
  return Array.from(document.querySelectorAll('.map-item-checkbox:checked')).map(
    (cb) => cb.dataset.mapName,
  );
}

function getSelectedWpLists() {
  return Array.from(document.querySelectorAll('.wp-list-checkbox:checked')).map(
    (cb) => cb.dataset.listName,
  );
}

function updateBatchHint() {
  const batchHint = document.getElementById('batchHint');
  const selectAllEl = document.getElementById('selectAll');
  const n = getSelected().length;
  const total = currentMaps.length;
  if (batchHint) batchHint.textContent = n ? `已选 ${n} / ${total}` : total ? `未选中（共 ${total}）` : '暂无地图';
  if (selectAllEl) {
    selectAllEl.checked = total > 0 && n === total;
    selectAllEl.indeterminate = n > 0 && n < total;
  }
}

function updateWpBatchHint() {
  const wpBatchHint = document.getElementById('wpBatchHint');
  const wpSelectAllEl = document.getElementById('wpSelectAll');
  const n = getSelectedWpLists().length;
  const total = pointListNames.length;
  if (wpBatchHint) {
    wpBatchHint.textContent = n
      ? `已选 ${n} / ${total}`
      : total
        ? `未选中（共 ${total}）`
        : '暂无导航点列表';
  }
  if (wpSelectAllEl) {
    wpSelectAllEl.checked = total > 0 && n === total;
    wpSelectAllEl.indeterminate = n > 0 && n < total;
  }
}

async function loadPointLists() {
  const j = await waypointManage(5);
  const names = j.names || [];
  pointListNames = Array.isArray(names) ? names.slice().sort() : [];
  pointListSet = new Set(pointListNames);
  pointListMeta = {};
  await Promise.all(
    pointListNames.map(async (listName) => {
      const r = await waypointManage(2, listName);
      let data = {};
      try {
        data =
          typeof r.data_json === 'string' ? JSON.parse(r.data_json || '{}') : r.data_json || {};
      } catch (_) {
        data = {};
      }
      const wps = Array.isArray(data.waypoints) ? data.waypoints : [];
      const hasCharger = wps.some((w) => String(w.name || '').toLowerCase() === 'charger');
      pointListMeta[listName] = { count: wps.length, hasCharger };
    }),
  );
}

function renderList() {
  const mapListEl = document.getElementById('mapList');
  const mapCountEl = document.getElementById('mapCount');
  if (!mapListEl) return;
  if (mapCountEl) mapCountEl.textContent = String(currentMaps.length);
  mapListEl.innerHTML = '';
  if (!currentMaps.length) {
    mapListEl.innerHTML = '<p class="muted pad">暂无地图文件</p>';
    updateBatchHint();
    return;
  }

  currentMaps.forEach((name, index) => {
    const wp = pointListNameFor(name);
    const hasWp = pointListSet.has(wp);
    const metaInfo = pointListMeta[wp];
    const row = document.createElement('div');
    row.className = 'map-item';
    const main = document.createElement('div');
    main.className = 'map-item-main';
    const cb = document.createElement('input');
    cb.type = 'checkbox';
    cb.className = 'map-item-checkbox';
    cb.dataset.mapName = name;
    cb.addEventListener('change', updateBatchHint);
    const badge = document.createElement('span');
    badge.className = 'index-badge';
    badge.textContent = String(index + 1);
    const info = document.createElement('div');
    const title = document.createElement('div');
    title.className = 'map-item-name';
    title.textContent = name;
    const meta = document.createElement('div');
    meta.className = 'map-item-meta ' + (hasWp ? 'has-wp' : 'no-wp');
    if (hasWp) {
      const n = metaInfo ? metaInfo.count : '?';
      const ch = metaInfo && metaInfo.hasCharger ? ' · 含 charger' : '';
      meta.textContent = `关联：${wp}（${n} 航点${ch}）`;
    } else {
      meta.textContent = `无关联 ${wp}`;
    }
    info.appendChild(title);
    info.appendChild(meta);
    main.appendChild(cb);
    main.appendChild(badge);
    main.appendChild(info);
    const actions = document.createElement('div');
    actions.className = 'map-item-actions';
    const renameBtn = document.createElement('button');
    renameBtn.type = 'button';
    renameBtn.className = 'secondary';
    renameBtn.textContent = '重命名';
    renameBtn.onclick = () => renameMap(name);
    const editBtn = document.createElement('button');
    editBtn.type = 'button';
    editBtn.className = 'secondary';
    editBtn.textContent = '美化';
    editBtn.onclick = () => navigateTo('/pages/map_beautify.html', `map=${encodeURIComponent(name)}`);
    const navBtn = document.createElement('button');
    navBtn.type = 'button';
    navBtn.className = 'secondary';
    navBtn.textContent = '导航';
    navBtn.onclick = () => navigateTo('/pages/navigation.html', `map=${encodeURIComponent(name)}`);
    const delBtn = document.createElement('button');
    delBtn.type = 'button';
    delBtn.className = 'secondary';
    delBtn.textContent = '删除';
    delBtn.onclick = () => deleteOne(name);
    actions.appendChild(renameBtn);
    actions.appendChild(editBtn);
    actions.appendChild(navBtn);
    actions.appendChild(delBtn);
    row.appendChild(main);
    row.appendChild(actions);
    mapListEl.appendChild(row);
  });
  updateBatchHint();
}

function renderWpCatalog() {
  const wpListCatalogEl = document.getElementById('wpListCatalog');
  const wpListCountEl = document.getElementById('wpListCount');
  if (!wpListCatalogEl) return;
  if (wpListCountEl) wpListCountEl.textContent = String(pointListNames.length);
  wpListCatalogEl.innerHTML = '';
  if (!pointListNames.length) {
    wpListCatalogEl.innerHTML = '<p class="muted pad">暂无导航点列表 · 建图保存时会自动创建</p>';
    updateWpBatchHint();
    return;
  }

  pointListNames.forEach((listName, index) => {
    const mapStem = mapNameFromPointList(listName);
    const mapExists = currentMaps.includes(mapStem);
    const metaInfo = pointListMeta[listName] || { count: 0, hasCharger: false };
    const row = document.createElement('div');
    row.className = 'map-item';
    const main = document.createElement('div');
    main.className = 'map-item-main';
    const cb = document.createElement('input');
    cb.type = 'checkbox';
    cb.className = 'wp-list-checkbox';
    cb.dataset.listName = listName;
    cb.addEventListener('change', updateWpBatchHint);
    const badge = document.createElement('span');
    badge.className = 'index-badge';
    badge.textContent = String(index + 1);
    const info = document.createElement('div');
    const title = document.createElement('div');
    title.className = 'map-item-name';
    title.textContent = listName;
    const meta = document.createElement('div');
    meta.className = 'map-item-meta ' + (mapExists ? 'has-wp' : 'no-wp');
    meta.textContent = [
      `${metaInfo.count} 航点`,
      metaInfo.hasCharger ? '含 charger' : '无 charger',
      mapExists ? `地图 ${mapStem}` : `缺地图 ${mapStem}`,
    ].join(' · ');
    info.appendChild(title);
    info.appendChild(meta);
    main.appendChild(cb);
    main.appendChild(badge);
    main.appendChild(info);
    const actions = document.createElement('div');
    actions.className = 'map-item-actions';
    const openBtn = document.createElement('button');
    openBtn.type = 'button';
    openBtn.className = 'secondary';
    openBtn.textContent = '打开编辑';
    openBtn.disabled = !mapExists;
    openBtn.onclick = () =>
      navigateTo('/pages/navigation.html', `map=${encodeURIComponent(mapStem)}`);
    const renameBtn = document.createElement('button');
    renameBtn.type = 'button';
    renameBtn.className = 'secondary';
    renameBtn.textContent = '重命名';
    renameBtn.onclick = () => renameWpList(listName);
    const delBtn = document.createElement('button');
    delBtn.type = 'button';
    delBtn.className = 'secondary';
    delBtn.textContent = '删除';
    delBtn.onclick = () => deleteWpListOne(listName);
    actions.appendChild(openBtn);
    actions.appendChild(renameBtn);
    actions.appendChild(delBtn);
    row.appendChild(main);
    row.appendChild(actions);
    wpListCatalogEl.appendChild(row);
  });
  updateWpBatchHint();
}

async function refresh() {
  setStatus('刷新中…');
  const mapsRes = await mapManage(2);
  currentMaps = mapsRes.map_list || [];
  if (!mapsRes.ok && !currentMaps.length) {
    setStatus(`地图列表失败：${mapsRes.message || 'bridge offline'}`, true);
  }
  try {
    await loadPointLists();
  } catch (e) {
    setStatus(`导航点列表失败：${e}`, true);
  }
  renderList();
  renderWpCatalog();
  setStatus('已刷新');
}

async function refreshWpOnly() {
  setStatus('刷新导航点列表…');
  try {
    await loadPointLists();
  } catch (e) {
    setStatus(`导航点列表失败：${e}`, true);
  }
  renderList();
  renderWpCatalog();
  setStatus('导航点列表已刷新');
}

async function renameMap(oldName) {
  const next = prompt('请输入新的地图名称：', oldName);
  if (next === null) return;
  const newName = next.trim();
  if (!newName || newName === oldName) return;
  setStatus(`重命名地图 ${oldName} → ${newName}`);
  const j = await mapManage(3, oldName, { new_name: newName });
  setStatus(j.ok ? j.message || '已重命名' : j.message || '重命名失败', !j.ok);
  await refresh();
}

async function renameWpList(oldName) {
  const mapStem = mapNameFromPointList(oldName);
  const next = prompt('请输入新的导航点列表名称（可写地图名或完整 xxx_pointList）：', mapStem || oldName);
  if (next === null) return;
  const newName = next.trim();
  if (!newName || newName === oldName || pointListNameFor(newName) === oldName) return;
  setStatus(`重命名列表 ${oldName} → ${newName}`);
  const j = await waypointManage(4, oldName, { new_name: newName });
  setStatus(j.ok ? j.message || '已重命名' : j.message || '重命名失败', !j.ok);
  await refresh();
}

async function deleteOne(name) {
  if (!confirm(`确定删除地图「${name}」及其关联 ${pointListNameFor(name)}？\n此操作不可撤销。`)) return;
  const j = await mapManage(4, name);
  setStatus(j.ok ? j.message || '已删除' : j.message || '删除失败', !j.ok);
  await refresh();
}

async function deleteWpListOne(listName) {
  if (!confirm(`确定删除导航点列表「${listName}」？\n只删航点文件，不会删除地图。\n此操作不可撤销。`)) return;
  const j = await waypointManage(3, listName);
  setStatus(j.ok ? j.message || '已删除' : j.message || '删除失败', !j.ok);
  await refresh();
}

async function deleteSequentially(names) {
  if (busy || !names.length) return;
  busy = true;
  let okN = 0;
  const failed = [];
  for (let i = 0; i < names.length; i++) {
    setStatus(`删除地图 ${i + 1}/${names.length}：${names[i]}`);
    const j = await mapManage(4, names[i]);
    if (j.ok) okN++;
    else failed.push(`${names[i]}: ${j.message || 'fail'}`);
  }
  busy = false;
  setStatus(!failed.length ? `成功删除 ${okN} 个地图` : `成功 ${okN}，失败 ${failed.length}`, !!failed.length);
  await refresh();
}

async function deleteWpListsSequentially(names) {
  if (wpBusy || !names.length) return;
  wpBusy = true;
  let okN = 0;
  const failed = [];
  for (let i = 0; i < names.length; i++) {
    setStatus(`删除列表 ${i + 1}/${names.length}：${names[i]}`);
    const j = await waypointManage(3, names[i]);
    if (j.ok) okN++;
    else failed.push(`${names[i]}: ${j.message || 'fail'}`);
  }
  wpBusy = false;
  setStatus(!failed.length ? `成功删除 ${okN} 个导航点列表` : `成功 ${okN}，失败 ${failed.length}`, !!failed.length);
  await refresh();
}

export function mount() {
  taskHandler = (l) => setStatus(l, /!!|fail|失败|错误/i.test(l));
  onTask(taskHandler);

  document.getElementById('refresh')?.addEventListener('click', refresh);
  document.getElementById('refreshWpLists')?.addEventListener('click', refreshWpOnly);

  const selectAllEl = document.getElementById('selectAll');
  selectAllEl?.addEventListener('change', () => {
    const on = selectAllEl.checked;
    document.querySelectorAll('.map-item-checkbox').forEach((cb) => {
      cb.checked = on;
    });
    updateBatchHint();
  });

  const wpSelectAllEl = document.getElementById('wpSelectAll');
  wpSelectAllEl?.addEventListener('change', () => {
    const on = wpSelectAllEl.checked;
    document.querySelectorAll('.wp-list-checkbox').forEach((cb) => {
      cb.checked = on;
    });
    updateWpBatchHint();
  });

  document.getElementById('batchDelete')?.addEventListener('click', () => {
    const selected = getSelected();
    if (!selected.length) return alert('请先勾选要删除的地图');
    if (!confirm(`确定删除选中的 ${selected.length} 个地图及其关联航点列表？\n此操作不可撤销。`)) return;
    deleteSequentially(selected);
  });

  document.getElementById('batchKeep')?.addEventListener('click', () => {
    const selected = getSelected();
    if (!selected.length) return alert('请先勾选要保留的地图');
    const toDelete = currentMaps.filter((n) => selected.indexOf(n) === -1);
    if (!toDelete.length) return alert('没有需要删除的地图（已全选保留）');
    if (!confirm(`将保留 ${selected.length} 个地图，删除其余 ${toDelete.length} 个。\n此操作不可撤销。`)) return;
    deleteSequentially(toDelete);
  });

  document.getElementById('wpBatchDelete')?.addEventListener('click', () => {
    const selected = getSelectedWpLists();
    if (!selected.length) return alert('请先勾选要删除的导航点列表');
    if (!confirm(`确定删除选中的 ${selected.length} 个导航点列表？\n只删航点文件，不删地图。\n此操作不可撤销。`)) return;
    deleteWpListsSequentially(selected);
  });

  document.getElementById('wpBatchKeep')?.addEventListener('click', () => {
    const selected = getSelectedWpLists();
    if (!selected.length) return alert('请先勾选要保留的导航点列表');
    const toDelete = pointListNames.filter((n) => selected.indexOf(n) === -1);
    if (!toDelete.length) return alert('没有需要删除的导航点列表（已全选保留）');
    if (!confirm(`将保留 ${selected.length} 个导航点列表，删除其余 ${toDelete.length} 个。\n此操作不可撤销。`)) return;
    deleteWpListsSequentially(toDelete);
  });

  refresh();
  return unmount;
}

export function unmount() {
  if (taskHandler) {
    offTask(taskHandler);
    taskHandler = null;
  }
  if (statusTimer) clearTimeout(statusTimer);
  currentMaps = [];
  pointListNames = [];
}
