/**
 * Gen2 map canvas — Foxglove WS draw /map + /scan + robot pose.
 * Also used by navigation: click-to-goal, waypoints, static PGM preview.
 * Depends on vendor: foxglove_bundle.js, roslib_foxglove.js, ros_ws_helper.js
 * Export: window.XwMapCanvas
 */
(function (global) {
  'use strict';

  // Scan lives in lidar_link (URDF yaw=π vs base_link). If map→lidar_link TF is
  // available, that rotation is already applied — do not add another π.
  // Extra π is only for base_link / initial-pose fallback (scan still in lidar_link).
  const LASER_DISPLAY_YAW_OFFSET = Math.PI;
  /** Waypoint / pose must stay this far from occupied cells (meters). */
  const OBSTACLE_CLEARANCE_M = 0.3;
  const OCCUPIED_THRESH = 50;

  // Soft edge only — keep cheap (1px dilate + light blur). Warm ivory glow, not cyan.
  const GLOW_CONFIG = {
    dilateRadius: 1,
    blurPx: 0.55,
    alpha: 0.28,
  };

  /** Default sensor frames relative to base_link (from xw_gen2.urdf). */
  const DEFAULT_SENSOR_FRAMES = [
    { id: 'lidar', frame: 'lidar_link', label: 'LiDAR', color: '#c23048' },
    { id: 'camera_front_up', frame: 'camera_front_up_link', label: '前上深度', color: '#22c55e' },
    { id: 'camera_front_down', frame: 'camera_front_down_link', label: '前下深度', color: '#16a34a' },
    { id: 'ultrasonic', frame: 'ultrasonic_front_link', label: '超声占位', color: '#f59e0b' },
  ];

  let mapCanvas = null;
  let overlayCanvas = null;
  let mapCtx = null;
  let overlayCtx = null;
  let containerEl = null;

  let ros = null;
  let mapTopic = null;
  let scanTopic = null;
  let planTopic = null;
  let tfTopic = null;
  let tfStaticTopic = null;

  let latestMap = null;
  let latestScan = null;
  let latestPlan = null; // nav_msgs/Path from /plan (keep last good)
  let latestLocalPlan = null; // nav_msgs/Path from /local_plan
  let localPlanTopic = null;
  let tfTree = {};
  const TF_MAX_AGE_NS = 10 * 1e9;

  let mapBitmapCanvas = null;
  let mapBitmapCtx = null;
  let mapBitmapKey = null;
  let glowBitmapCanvas = null;
  let glowBitmapCtx = null;
  let dilatedGlowCanvas = null;
  let dilatedGlowCtx = null;

  let started = false;
  let overlayTimer = null;
  let tfCleanTimer = null;
  let resizeObserver = null;
  let statusCb = null;
  let clickCb = null;
  let modeChangeCb = null;
  let interactive = false;
  let preferLiveMap = true;
  let showSensorFrames = false;
  let goalPose = null; // { x, y, yaw }
  let stagingPose = null; // recharge approach pose { x, y, yaw }
  let waypoints = []; // [{ name, x, y, yaw?, bad? }]
  let scanAgeMs = 0;
  let lastScanTs = 0;
  let pointerBound = false;

  /** 'view' | 'initial_pose' | 'edit_wp' | 'goal'(legacy) */
  let interactMode = 'view';
  let waypointClickCb = null;
  let initialPose = null; // { x, y, yaw }
  let selectedWpIdx = null;
  let dragState = null; // { type: 'pose'|'wp', idx?, ox, oy }
  let yawChangeCb = null;

  function setStatus(msg) {
    if (typeof statusCb === 'function') statusCb(msg);
  }

  function readOccupancyCell(data, index) {
    if (!data) return -1;
    let value = data[index];
    if (value === undefined && typeof data.get === 'function') {
      value = data.get(index);
    }
    if (typeof value !== 'number' || Number.isNaN(value)) return -1;
    if (value > 127) value -= 256;
    return value;
  }

  function normalizeOccupancyGridMessage(message) {
    if (!message || typeof message !== 'object') return message;
    const normalized = { ...message };
    if (message.header && typeof message.header === 'object') {
      normalized.header = { ...message.header };
      if (message.header.stamp && typeof message.header.stamp === 'object') {
        normalized.header.stamp = {
          sec: Number(message.header.stamp.sec || 0),
          nanosec: Number(message.header.stamp.nanosec || 0),
        };
      }
    }
    if (message.info && typeof message.info === 'object') {
      normalized.info = {
        ...message.info,
        width: Number(message.info.width || 0),
        height: Number(message.info.height || 0),
        resolution: Number(message.info.resolution || 0),
      };
    }
    const data = message.data;
    if (Array.isArray(data) || ArrayBuffer.isView(data)) {
      normalized.data = data;
      return normalized;
    }
    if (data && typeof data === 'object') {
      const numericKeys = Object.keys(data).filter((k) => /^\d+$/.test(k));
      if (numericKeys.length > 0) {
        numericKeys.sort((a, b) => Number(a) - Number(b));
        normalized.data = numericKeys.map((k) => Number(data[k]));
      }
    }
    return normalized;
  }

  function ensureMapBitmapCanvas(width, height) {
    if (!mapBitmapCanvas || mapBitmapCanvas.width !== width || mapBitmapCanvas.height !== height) {
      mapBitmapCanvas = document.createElement('canvas');
      mapBitmapCanvas.width = width;
      mapBitmapCanvas.height = height;
      mapBitmapCtx = mapBitmapCanvas.getContext('2d', { willReadFrequently: false });
    }
  }

  function ensureGlowBitmapCanvas(width, height) {
    if (!glowBitmapCanvas || glowBitmapCanvas.width !== width || glowBitmapCanvas.height !== height) {
      glowBitmapCanvas = document.createElement('canvas');
      glowBitmapCanvas.width = width;
      glowBitmapCanvas.height = height;
      glowBitmapCtx = glowBitmapCanvas.getContext('2d', { willReadFrequently: false });
    }
  }

  function ensureDilatedGlowCanvas(width, height) {
    if (!dilatedGlowCanvas || dilatedGlowCanvas.width !== width || dilatedGlowCanvas.height !== height) {
      dilatedGlowCanvas = document.createElement('canvas');
      dilatedGlowCanvas.width = width;
      dilatedGlowCanvas.height = height;
      dilatedGlowCtx = dilatedGlowCanvas.getContext('2d', { willReadFrequently: false });
    }
  }

  function buildMapBitmap(message) {
    if (!message || !message.info) return;
    const width = message.info.width;
    const height = message.info.height;
    if (!width || !height) return;

    const sec = message.header && message.header.stamp ? message.header.stamp.sec : 0;
    const nsec = message.header && message.header.stamp ? message.header.stamp.nanosec : 0;
    const bitmapKey = `${width}x${height}:${sec}:${nsec}`;
    if (bitmapKey === mapBitmapKey) return;

    ensureMapBitmapCanvas(width, height);
    ensureGlowBitmapCanvas(width, height);
    ensureDilatedGlowCanvas(width, height);

    const imageData = mapBitmapCtx.createImageData(width, height);
    const pixels = imageData.data;
    const glowImageData = glowBitmapCtx.createImageData(width, height);
    const glowPixels = glowImageData.data;
    const dilatedImageData = dilatedGlowCtx.createImageData(width, height);
    const dilatedPixels = dilatedImageData.data;
    const rowStride = width * 4;
    const dilateR = GLOW_CONFIG.dilateRadius;

    for (let y = 0; y < height; y++) {
      const srcRow = y * width;
      const dstRow = (height - 1 - y) * rowStride;
      const dstY = height - 1 - y;
      for (let x = 0; x < width; x++) {
        const value = readOccupancyCell(message.data, srcRow + x);
        const pixelOffset = dstRow + x * 4;
        if (value === -1) {
          // Unknown: deep ink (neutral, no purple cast)
          pixels[pixelOffset] = 20;
          pixels[pixelOffset + 1] = 22;
          pixels[pixelOffset + 2] = 26;
          pixels[pixelOffset + 3] = 255;
          glowPixels[pixelOffset + 3] = 0;
        } else if (value === 0) {
          // Free: warm slate floor
          pixels[pixelOffset] = 46;
          pixels[pixelOffset + 1] = 50;
          pixels[pixelOffset + 2] = 56;
          pixels[pixelOffset + 3] = 255;
          glowPixels[pixelOffset + 3] = 0;
        } else {
          // Occupied: soft taupe → warm ivory (architectural, not ice-blue)
          const t = value / 100;
          const wr = Math.round(118 + t * 118);
          const wg = Math.round(108 + t * 118);
          const wb = Math.round(96 + t * 112);
          glowPixels[pixelOffset] = wr;
          glowPixels[pixelOffset + 1] = wg;
          glowPixels[pixelOffset + 2] = wb;
          glowPixels[pixelOffset + 3] = 255;
          for (let dy = -dilateR; dy <= dilateR; dy++) {
            const ny = dstY + dy;
            if (ny < 0 || ny >= height) continue;
            const dilRow = ny * rowStride;
            for (let dx = -dilateR; dx <= dilateR; dx++) {
              const nx = x + dx;
              if (nx < 0 || nx >= width) continue;
              const doff = dilRow + nx * 4;
              pixels[doff] = wr;
              pixels[doff + 1] = wg;
              pixels[doff + 2] = wb;
              pixels[doff + 3] = 255;
              dilatedPixels[doff] = wr;
              dilatedPixels[doff + 1] = wg;
              dilatedPixels[doff + 2] = wb;
              dilatedPixels[doff + 3] = 255;
            }
          }
        }
      }
    }

    mapBitmapCtx.putImageData(imageData, 0, 0);
    glowBitmapCtx.putImageData(glowImageData, 0, 0);
    dilatedGlowCtx.putImageData(dilatedImageData, 0, 0);
    mapBitmapKey = bitmapKey;
  }

  function getYawFromQuaternion(q) {
    return Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  function updateTfTree(msg) {
    if (!msg || !msg.transforms) return;
    msg.transforms.forEach((transform) => {
      const parent = String(transform.header.frame_id || '').replace(/^\//, '');
      const child = String(transform.child_frame_id || '').replace(/^\//, '');
      const key = `${parent}->${child}`;
      const sec = transform.header.stamp.sec;
      const nsec = transform.header.stamp.nanosec;
      const newStamp = sec * 1e9 + nsec;
      if (!tfTree[key] || newStamp > tfTree[key].stamp) {
        tfTree[key] = { parent, child, transform, stamp: newStamp };
      }
    });
  }

  function getTransform(from, to, visited) {
    if (from === to) return { x: 0, y: 0, yaw: 0 };
    visited = visited || new Set();
    visited.add(to);
    for (const key in tfTree) {
      const link = tfTree[key];
      if (link.child === to && !visited.has(link.parent)) {
        const t = link.transform.transform.translation;
        const q = link.transform.transform.rotation;
        const linkYaw = getYawFromQuaternion(q);
        const parentTf = getTransform(from, link.parent, visited);
        if (parentTf) {
          const tx = Math.cos(parentTf.yaw) * t.x - Math.sin(parentTf.yaw) * t.y;
          const ty = Math.sin(parentTf.yaw) * t.x + Math.cos(parentTf.yaw) * t.y;
          return {
            x: parentTf.x + tx,
            y: parentTf.y + ty,
            yaw: parentTf.yaw + linkYaw,
          };
        }
      }
    }
    return null;
  }

  function getRobotTf() {
    const frames = ['base_link', 'base_footprint', 'base_scan'];
    for (let i = 0; i < frames.length; i++) {
      const t = getTransform('map', frames[i]);
      if (t) return t;
    }
    return null;
  }

  function chooseGridSpacing(pxPerMeter) {
    const targetPx = 70;
    const raw = targetPx / Math.max(pxPerMeter, 0.01);
    const nice = [0.5, 1, 2, 5, 10, 20, 50, 100];
    for (const n of nice) {
      if (raw <= n) return n;
    }
    return nice[nice.length - 1];
  }

  /** @returns {boolean} true if canvas buffer size changed (needs full map redraw). */
  function resizeCanvases() {
    if (!containerEl || !mapCanvas || !overlayCanvas) return false;
    const width = containerEl.clientWidth;
    const height = containerEl.clientHeight;
    if (!width || !height) return false;
    if (mapCanvas.width !== width || mapCanvas.height !== height) {
      mapCanvas.width = width;
      mapCanvas.height = height;
      overlayCanvas.width = width;
      overlayCanvas.height = height;
      return true;
    }
    return false;
  }

  function onContainerResize() {
    if (resizeCanvases()) {
      redraw();
    }
  }

  function drawMap() {
    if (!latestMap || !mapBitmapCanvas || !mapCtx) return;
    if (!mapCanvas.width || !mapCanvas.height) {
      if (!resizeCanvases()) return;
    }
    mapCtx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);

    const info = latestMap.info;
    const mapRes = info.resolution;
    const scaleX = mapCanvas.width / info.width;
    const scaleY = mapCanvas.height / info.height;
    const scale = Math.min(scaleX, scaleY);
    const drawW = info.width * scale;
    const drawH = info.height * scale;
    const offsetX = (mapCanvas.width - drawW) / 2;
    const offsetY = (mapCanvas.height - drawH) / 2;

    if (dilatedGlowCanvas) {
      mapCtx.save();
      mapCtx.filter = 'blur(' + Math.max(1, scale * GLOW_CONFIG.blurPx) + 'px)';
      mapCtx.globalAlpha = GLOW_CONFIG.alpha;
      mapCtx.drawImage(dilatedGlowCanvas, offsetX, offsetY, drawW, drawH);
      mapCtx.restore();
    }

    mapCtx.imageSmoothingEnabled = false;
    mapCtx.drawImage(mapBitmapCanvas, offsetX, offsetY, drawW, drawH);

    const pxPerMeter = scale / mapRes;
    const spacing = chooseGridSpacing(pxPerMeter);
    const worldLeft = info.origin.position.x;
    const worldTop = info.origin.position.y + info.height * mapRes;
    const worldRight = worldLeft + info.width * mapRes;
    const worldBottom = info.origin.position.y;
    const gridStartX = Math.floor(worldLeft / spacing) * spacing;
    const gridStartY = Math.floor(worldBottom / spacing) * spacing;

    mapCtx.save();
    mapCtx.strokeStyle = 'rgba(232, 224, 208, 0.07)';
    mapCtx.lineWidth = 0.5;
    for (let wx = gridStartX; wx <= worldRight; wx += spacing) {
      const sx = offsetX + ((wx - worldLeft) / mapRes) * scale;
      mapCtx.beginPath();
      mapCtx.moveTo(sx, offsetY);
      mapCtx.lineTo(sx, offsetY + drawH);
      mapCtx.stroke();
    }
    for (let wy = gridStartY; wy <= worldTop; wy += spacing) {
      const sy = offsetY + ((worldTop - wy) / mapRes) * scale;
      mapCtx.beginPath();
      mapCtx.moveTo(offsetX, sy);
      mapCtx.lineTo(offsetX + drawW, sy);
      mapCtx.stroke();
    }
    mapCtx.restore();
  }

  function getViewTransform() {
    if (!latestMap || !overlayCanvas) return null;
    const scaleX = overlayCanvas.width / latestMap.info.width;
    const scaleY = overlayCanvas.height / latestMap.info.height;
    const scale = Math.min(scaleX, scaleY);
    return {
      scale: scale,
      offsetX: (overlayCanvas.width - latestMap.info.width * scale) / 2,
      offsetY: (overlayCanvas.height - latestMap.info.height * scale) / 2,
      origin: latestMap.info.origin,
      mapRes: latestMap.info.resolution,
      height: latestMap.info.height,
      width: latestMap.info.width,
    };
  }

  function worldToPixel(wx, wy, view) {
    const mx = (wx - view.origin.position.x) / view.mapRes;
    const my = (wy - view.origin.position.y) / view.mapRes;
    return {
      px: view.offsetX + mx * view.scale,
      py: view.offsetY + (view.height - my) * view.scale,
    };
  }

  function canvasToWorld(clientX, clientY) {
    if (!latestMap || !overlayCanvas) return null;
    const rect = overlayCanvas.getBoundingClientRect();
    const cx = ((clientX - rect.left) / rect.width) * overlayCanvas.width;
    const cy = ((clientY - rect.top) / rect.height) * overlayCanvas.height;
    const view = getViewTransform();
    if (!view || !view.scale) return null;
    const mx = (cx - view.offsetX) / view.scale;
    const my = view.height - (cy - view.offsetY) / view.scale;
    return {
      x: view.origin.position.x + mx * view.mapRes,
      y: view.origin.position.y + my * view.mapRes,
    };
  }

  function worldToMapCell(wx, wy) {
    if (!latestMap || !latestMap.info) return null;
    const info = latestMap.info;
    const mx = Math.floor((wx - info.origin.position.x) / info.resolution);
    const my = Math.floor((wy - info.origin.position.y) / info.resolution);
    if (mx < 0 || my < 0 || mx >= info.width || my >= info.height) return null;
    return { mx: mx, my: my, width: info.width, height: info.height, res: info.resolution };
  }

  function cellOccupied(mx, my) {
    if (!latestMap) return true;
    const idx = my * latestMap.info.width + mx;
    const v = readOccupancyCell(latestMap.data, idx);
    return v >= OCCUPIED_THRESH;
  }

  /** True if point is at least clearanceM from any occupied cell. No map → unknown (true). */
  function isClearanceOk(wx, wy, clearanceM) {
    if (!latestMap) return true;
    const clr = typeof clearanceM === 'number' ? clearanceM : OBSTACLE_CLEARANCE_M;
    const cell = worldToMapCell(wx, wy);
    if (!cell) return false;
    if (cellOccupied(cell.mx, cell.my)) return false;
    const r = Math.max(1, Math.ceil(clr / cell.res));
    for (let dy = -r; dy <= r; dy++) {
      for (let dx = -r; dx <= r; dx++) {
        if (dx === 0 && dy === 0) continue;
        const dist = Math.sqrt(dx * dx + dy * dy) * cell.res;
        if (dist > clr + 1e-6) continue;
        const nx = cell.mx + dx;
        const ny = cell.my + dy;
        if (nx < 0 || ny < 0 || nx >= cell.width || ny >= cell.height) continue;
        if (cellOccupied(nx, ny)) return false;
      }
    }
    return true;
  }

  function annotateWaypointBadness(list) {
    return (list || []).map((wp) => {
      if (!wp || typeof wp.x !== 'number') return wp;
      if (!latestMap) return Object.assign({}, wp, { bad: false });
      const ok = isClearanceOk(wp.x, wp.y, OBSTACLE_CLEARANCE_M);
      return Object.assign({}, wp, { bad: !ok });
    });
  }

  function pixelDist(clientX, clientY, wx, wy, view) {
    const rect = overlayCanvas.getBoundingClientRect();
    const cx = ((clientX - rect.left) / rect.width) * overlayCanvas.width;
    const cy = ((clientY - rect.top) / rect.height) * overlayCanvas.height;
    const p = worldToPixel(wx, wy, view);
    const dx = cx - p.px;
    const dy = cy - p.py;
    return Math.sqrt(dx * dx + dy * dy);
  }

  function fxMode() {
    return (document.documentElement && document.documentElement.dataset.fx) || 'high';
  }

  function drawPoseMarker(view, x, y, yaw, fill, stroke, radius) {
    const p = worldToPixel(x, y, view);
    const r = radius || Math.max(7, view.scale * 1.8);
    const fx = fxMode();
    if (fx !== 'off') {
      const pulse = 0.55 + 0.45 * Math.sin(Date.now() / 420);
      const haloR = r + 6 + pulse * 5;
      overlayCtx.beginPath();
      overlayCtx.arc(p.px, p.py, haloR, 0, 2 * Math.PI);
      overlayCtx.strokeStyle = 'rgba(34, 211, 238, ' + (0.18 + pulse * 0.2) + ')';
      overlayCtx.lineWidth = 2;
      overlayCtx.stroke();
      if (fx === 'high') {
        overlayCtx.beginPath();
        overlayCtx.arc(p.px, p.py, haloR + 4, 0, 2 * Math.PI);
        overlayCtx.strokeStyle = 'rgba(232, 121, 249, ' + (0.08 + pulse * 0.12) + ')';
        overlayCtx.lineWidth = 1.5;
        overlayCtx.stroke();
      }
    }
    overlayCtx.beginPath();
    overlayCtx.arc(p.px, p.py, r, 0, 2 * Math.PI);
    overlayCtx.fillStyle = fill;
    overlayCtx.fill();
    if (stroke) {
      overlayCtx.strokeStyle = stroke;
      overlayCtx.lineWidth = 2;
      overlayCtx.stroke();
    }
    if (typeof yaw === 'number' && !Number.isNaN(yaw)) {
      const arrowLen = Math.max(14, view.scale * 3.5);
      overlayCtx.beginPath();
      overlayCtx.moveTo(p.px, p.py);
      overlayCtx.lineTo(p.px + arrowLen * Math.cos(yaw), p.py - arrowLen * Math.sin(yaw));
      overlayCtx.strokeStyle = stroke || fill;
      overlayCtx.lineWidth = 2.5;
      overlayCtx.stroke();
    }
    return p;
  }

  /** Navigation waypoints — larger, themed markers with halo + label. */
  function drawWaypointMarker(view, wp, idx, opts) {
    opts = opts || {};
    const bad = !!opts.bad;
    const selected = !!opts.selected;
    const isCharger = !!opts.isCharger;
    const p = worldToPixel(wp.x, wp.y, view);
    const r = Math.max(11, view.scale * 2.6);

    let fill;
    let stroke;
    let halo;
    if (bad) {
      fill = 'rgba(192, 51, 69, 0.95)';
      stroke = '#8f2433';
      halo = 'rgba(192, 51, 69, 0.28)';
    } else if (selected) {
      fill = 'rgba(217, 145, 32, 0.96)';
      stroke = '#a36b10';
      halo = 'rgba(217, 145, 32, 0.32)';
    } else if (isCharger) {
      fill = 'rgba(15, 122, 79, 0.94)';
      stroke = '#0a5c3c';
      halo = 'rgba(15, 122, 79, 0.26)';
    } else {
      fill = 'rgba(12, 110, 138, 0.94)';
      stroke = '#0a5c74';
      halo = 'rgba(12, 110, 138, 0.26)';
    }

    overlayCtx.beginPath();
    overlayCtx.arc(p.px, p.py, r + 5, 0, 2 * Math.PI);
    overlayCtx.fillStyle = halo;
    overlayCtx.fill();

    overlayCtx.beginPath();
    overlayCtx.arc(p.px, p.py, r + 1.5, 0, 2 * Math.PI);
    overlayCtx.fillStyle = 'rgba(255, 255, 255, 0.92)';
    overlayCtx.fill();

    overlayCtx.beginPath();
    overlayCtx.arc(p.px, p.py, r, 0, 2 * Math.PI);
    overlayCtx.fillStyle = fill;
    overlayCtx.fill();
    overlayCtx.strokeStyle = stroke;
    overlayCtx.lineWidth = 2.25;
    overlayCtx.stroke();

    if (typeof wp.yaw === 'number' && !Number.isNaN(wp.yaw)) {
      const arrowLen = Math.max(20, view.scale * 4.2);
      const tipX = p.px + arrowLen * Math.cos(wp.yaw);
      const tipY = p.py - arrowLen * Math.sin(wp.yaw);
      overlayCtx.beginPath();
      overlayCtx.moveTo(p.px, p.py);
      overlayCtx.lineTo(tipX, tipY);
      overlayCtx.strokeStyle = stroke;
      overlayCtx.lineWidth = 3;
      overlayCtx.lineCap = 'round';
      overlayCtx.stroke();
      overlayCtx.beginPath();
      overlayCtx.arc(tipX, tipY, 3.2, 0, 2 * Math.PI);
      overlayCtx.fillStyle = stroke;
      overlayCtx.fill();
    }

    overlayCtx.fillStyle = '#fff';
    overlayCtx.font = 'bold 13px "IBM Plex Mono", ui-monospace, sans-serif';
    overlayCtx.textAlign = 'center';
    overlayCtx.textBaseline = 'middle';
    overlayCtx.fillText(bad ? '!' : isCharger ? 'C' : String(idx + 1), p.px, p.py + 0.5);

    const labelY = p.py - r - 10;
    overlayCtx.font = '600 11px "Noto Sans SC", sans-serif';
    overlayCtx.textBaseline = 'bottom';
    const label = bad ? '坏点' : String(wp.name || (isCharger ? '充电桩' : `wp_${idx + 1}`));
    const tw = overlayCtx.measureText(label).width;
    const padX = 5;
    const padY = 3;
    overlayCtx.fillStyle = 'rgba(255, 255, 255, 0.88)';
    overlayCtx.strokeStyle = 'rgba(11, 18, 32, 0.08)';
    overlayCtx.lineWidth = 1;
    const bx = p.px - tw / 2 - padX;
    const by = labelY - 12 - padY;
    const bw = tw + padX * 2;
    const bh = 14 + padY;
    overlayCtx.beginPath();
    if (overlayCtx.roundRect) {
      overlayCtx.roundRect(bx, by, bw, bh, 4);
    } else {
      overlayCtx.rect(bx, by, bw, bh);
    }
    overlayCtx.fill();
    overlayCtx.stroke();
    overlayCtx.fillStyle = bad ? '#c03345' : selected ? '#a36b10' : '#0c6e8a';
    overlayCtx.fillText(label, p.px, labelY);

    return p;
  }

  /** Foxglove/CDR sometimes yields sequences as {0:…,1:…} instead of Array. */
  function coercePoseArray(poses) {
    if (Array.isArray(poses)) return poses;
    if (!poses || typeof poses !== 'object') return null;
    const keys = Object.keys(poses).filter((k) => /^\d+$/.test(k));
    if (!keys.length) return null;
    keys.sort((a, b) => Number(a) - Number(b));
    return keys.map((k) => poses[k]);
  }

  function normalizePathMessage(msg) {
    if (!msg || typeof msg !== 'object') return null;
    const poses = coercePoseArray(msg.poses);
    if (!poses) return null;
    return { header: msg.header, poses: poses };
  }

  function strokePath(view, poses, strokeStyle, lineWidth, alpha) {
    if (!poses || poses.length < 2) return;
    const first = poses[0] && poses[0].pose && poses[0].pose.position;
    if (!first || typeof first.x !== 'number') return;
    overlayCtx.save();
    overlayCtx.beginPath();
    const p0 = worldToPixel(first.x, first.y, view);
    overlayCtx.moveTo(p0.px, p0.py);
    for (let i = 1; i < poses.length; i++) {
      const pos = poses[i] && poses[i].pose && poses[i].pose.position;
      if (!pos || typeof pos.x !== 'number') continue;
      const pt = worldToPixel(pos.x, pos.y, view);
      overlayCtx.lineTo(pt.px, pt.py);
    }
    overlayCtx.strokeStyle = strokeStyle;
    overlayCtx.lineWidth = lineWidth;
    overlayCtx.lineJoin = 'round';
    overlayCtx.lineCap = 'round';
    overlayCtx.globalAlpha = alpha;
    overlayCtx.stroke();
    overlayCtx.restore();
  }

  function drawPlan(view) {
    if (latestPlan && latestPlan.poses && latestPlan.poses.length >= 2) {
      strokePath(
        view,
        latestPlan.poses,
        '#00bcd4',
        Math.max(3, view.scale * 0.9),
        0.78,
      );
    }
    if (latestLocalPlan && latestLocalPlan.poses && latestLocalPlan.poses.length >= 2) {
      strokePath(
        view,
        latestLocalPlan.poses,
        '#f59e0b',
        Math.max(2.2, view.scale * 0.7),
        0.9,
      );
    }
  }

  function drawOverlay() {
    if (!latestMap || !overlayCtx) return;
    overlayCtx.clearRect(0, 0, overlayCanvas.width, overlayCanvas.height);
    const view = getViewTransform();
    if (!view) return;

    const robotTf = getRobotTf();
    const displayRobot =
      interactMode === 'initial_pose' && initialPose
        ? initialPose
        : initialPose && !robotTf
          ? initialPose
          : robotTf;

    // Global plan under markers / scan so path stays visible.
    drawPlan(view);

    if (latestScan) {
      let tf = null;
      let usedScanFrame = false;
      // While dragging initial pose, attach scan to the preview pose so lidar follows the ghost robot.
      if (interactMode === 'initial_pose' && initialPose) {
        tf = initialPose;
        usedScanFrame = false;
      } else if (latestScan.header && latestScan.header.frame_id) {
        const fid = String(latestScan.header.frame_id).replace(/^\//, '');
        tf = getTransform('map', fid);
        if (tf) usedScanFrame = true;
      }
      if (!tf) tf = robotTf || (initialPose ? initialPose : null);
      if (tf) {
        const robotX = tf.x;
        const robotY = tf.y;
        const extraYaw = usedScanFrame ? 0 : LASER_DISPLAY_YAW_OFFSET;
        const scanYaw = (typeof tf.yaw === 'number' ? tf.yaw : 0) + extraYaw;
        const fx = fxMode();
        // Laser as sole cool accent against warm ivory walls
        const laserColor = fx === 'off' ? '#e85d4c' : '#3dd6c6';
        overlayCtx.fillStyle = laserColor;
        const ranges = latestScan.ranges || [];
        const step = fx === 'high' ? 1 : fx === 'low' ? 2 : 3;
        for (let i = 0; i < ranges.length; i += step) {
          const range = ranges[i];
          if (range < latestScan.range_min || range > latestScan.range_max) continue;
          const angle = latestScan.angle_min + i * latestScan.angle_increment;
          const lx = range * Math.cos(angle);
          const ly = range * Math.sin(angle);
          const wx = robotX + Math.cos(scanYaw) * lx - Math.sin(scanYaw) * ly;
          const wy = robotY + Math.sin(scanYaw) * lx + Math.cos(scanYaw) * ly;
          const p = worldToPixel(wx, wy, view);
          overlayCtx.beginPath();
          overlayCtx.arc(p.px, p.py, Math.max(2.2, view.scale * 0.85), 0, 2 * Math.PI);
          overlayCtx.fill();
        }
      }
    }

    if (Array.isArray(waypoints)) {
      waypoints.forEach((wp, idx) => {
        if (!wp || typeof wp.x !== 'number') return;
        const bad = latestMap
          ? !!wp.bad || !isClearanceOk(wp.x, wp.y, OBSTACLE_CLEARANCE_M)
          : false;
        wp.bad = bad;
        const selected = selectedWpIdx === idx;
        const isCharger =
          wp._kind === 'charger' ||
          String(wp.name || '').toLowerCase() === 'charger';
        drawWaypointMarker(view, wp, idx, { bad, selected, isCharger });
      });
    }

    if (stagingPose && typeof stagingPose.x === 'number') {
      drawPoseMarker(
        view,
        stagingPose.x,
        stagingPose.y,
        typeof stagingPose.yaw === 'number' ? stagingPose.yaw : 0,
        'rgba(5, 150, 105, 0.92)',
        '#047857',
        Math.max(7, view.scale * 1.7)
      );
      const sp = worldToPixel(stagingPose.x, stagingPose.y, view);
      overlayCtx.fillStyle = '#047857';
      overlayCtx.font = '11px "IBM Plex Mono", sans-serif';
      overlayCtx.textAlign = 'center';
      overlayCtx.fillText('接近点', sp.px, sp.py - Math.max(14, view.scale * 2.6));
    }

    if (goalPose && typeof goalPose.x === 'number') {
      drawPoseMarker(
        view,
        goalPose.x,
        goalPose.y,
        typeof goalPose.yaw === 'number' ? goalPose.yaw : 0,
        'rgba(34, 197, 94, 0.9)',
        '#15803d',
        Math.max(8, view.scale * 2)
      );
    }

    if (displayRobot) {
      const isPreview = !robotTf || (interactMode === 'initial_pose' && initialPose);
      drawPoseMarker(
        view,
        displayRobot.x,
        displayRobot.y,
        typeof displayRobot.yaw === 'number' ? displayRobot.yaw : 0,
        isPreview ? 'rgba(34, 211, 238, 0.92)' : 'rgba(103, 232, 249, 0.9)',
        isPreview ? '#0891b2' : '#22d3ee',
        Math.max(9, view.scale * 2.2)
      );
      if (isPreview) {
        const p = worldToPixel(displayRobot.x, displayRobot.y, view);
        overlayCtx.fillStyle = '#67e8f9';
        overlayCtx.font = '11px "IBM Plex Mono", sans-serif';
        overlayCtx.textAlign = 'center';
        overlayCtx.fillText('初位姿', p.px, p.py + Math.max(16, view.scale * 3));
      }
    }

    if (showSensorFrames && robotTf) {
      DEFAULT_SENSOR_FRAMES.forEach((sf) => {
        let tf = getTransform('map', sf.frame);
        if (!tf) {
          const rel = getTransform('base_link', sf.frame);
          if (rel && robotTf) {
            const cos = Math.cos(robotTf.yaw);
            const sin = Math.sin(robotTf.yaw);
            tf = {
              x: robotTf.x + cos * rel.x - sin * rel.y,
              y: robotTf.y + sin * rel.x + cos * rel.y,
              yaw: robotTf.yaw + rel.yaw,
            };
          }
        }
        if (!tf) return;
        const p = worldToPixel(tf.x, tf.y, view);
        overlayCtx.beginPath();
        overlayCtx.arc(p.px, p.py, Math.max(3, view.scale * 0.8), 0, 2 * Math.PI);
        overlayCtx.fillStyle = sf.color;
        overlayCtx.fill();
      });
    }
  }

  function redraw() {
    resizeCanvases();
    drawMap();
    drawOverlay();
  }

  function applyMapMessage(message, statusText) {
    const normalized = normalizeOccupancyGridMessage(message);
    if (!normalized || !normalized.info || !normalized.info.width || !normalized.info.height) {
      return false;
    }
    latestMap = normalized;
    mapBitmapKey = null;
    buildMapBitmap(normalized);
    redraw();
    if (statusText) setStatus(statusText);
    return true;
  }

  function pgmByteToOccupancy(v) {
    if (v >= 250) return 0;
    if (v <= 50) return 100;
    if (Math.abs(v - 205) <= 25) return -1;
    return Math.max(0, Math.min(100, Math.round(((255 - v) * 100) / 255)));
  }

  function b64ToBytes(b64) {
    const bin = atob(b64);
    const out = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
    return out;
  }

  /**
   * Load saved map from /api/map op=5 payload (pgm_b64).
   * opts.allowLive — keep accepting /map (nav session / refresh). Default false
   * so idle map preview is not overwritten by a stale live grid.
   */
  function loadStaticMap(payload, opts) {
    opts = opts || {};
    if (!payload || !payload.pgm_b64 || !payload.width || !payload.height) {
      setStatus('静态地图数据无效');
      return false;
    }
    const width = Number(payload.width) | 0;
    const height = Number(payload.height) | 0;
    const bytes = b64ToBytes(payload.pgm_b64);
    if (bytes.length !== width * height) {
      setStatus(`静态地图像素不匹配 ${bytes.length}≠${width * height}`);
      return false;
    }
    const data = new Int8Array(width * height);
    for (let i = 0; i < bytes.length; i++) {
      data[i] = pgmByteToOccupancy(bytes[i]);
    }
    const allowLive = !!opts.allowLive;
    // Already showing a live /map during nav: keep it; static is only a fallback.
    if (
      allowLive &&
      preferLiveMap &&
      latestMap &&
      latestMap.header &&
      latestMap.header.stamp &&
      Number(latestMap.header.stamp.sec || 0) > 0
    ) {
      setStatus(`保留实时地图 · 静态 ${width}×${height} 作后备`);
      return true;
    }
    preferLiveMap = allowLive;
    const ox = Number(payload.origin_x);
    const oy = Number(payload.origin_y);
    const msg = {
      header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'map' },
      info: {
        resolution: Number(payload.resolution) || 0.05,
        width: width,
        height: height,
        origin: {
          position: {
            x: Number.isFinite(ox) ? ox : 0,
            y: Number.isFinite(oy) ? oy : 0,
            z: 0,
          },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
      },
      data: data,
    };
    return applyMapMessage(
      msg,
      allowLive
        ? `静态底图已加载 · 等待 /map · ${width}×${height}`
        : `静态地图已加载 · ${width}×${height}`,
    );
  }

  /** Re-subscribe /map so foxglove re-delivers the latched OccupancyGrid. */
  function resubscribeMapTopic() {
    if (!ros || typeof ROSLIB === 'undefined') return;
    try {
      if (mapTopic && typeof mapTopic.unsubscribe === 'function') {
        mapTopic.unsubscribe();
      }
    } catch (_) {
      /* ignore */
    }
    mapTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/map',
      messageType: 'nav_msgs/msg/OccupancyGrid',
    });
    mapTopic.subscribe(onMapMessage);
  }

  function hasLiveMap() {
    return !!(
      latestMap &&
      latestMap.header &&
      latestMap.header.stamp &&
      Number(latestMap.header.stamp.sec || 0) > 0
    );
  }

  /**
   * Prefer /map over static preview.
   * opts.forceResub — unsubscribe/resubscribe to re-pull latched OccupancyGrid
   * (needed after refresh when the only sample was ignored while preferLiveMap=false).
   */
  function enableLiveMap(opts) {
    opts = opts || {};
    preferLiveMap = true;
    const needResub = !!opts.forceResub || !hasLiveMap();
    if (needResub) {
      setStatus('等待 /map 实时话题…（已保留静态底图直至收到）');
    }
    if (started && ros && needResub) {
      resubscribeMapTopic();
    }
  }

  /** Keep showing static until first live /map; then switch. */
  function onMapMessage(message) {
    if (!preferLiveMap) return;
    try {
      if (!applyMapMessage(message, '地图已更新（/map 实时）')) return;
    } catch (err) {
      console.error('[XwMapCanvas] /map handler error', err);
      setStatus('地图解析失败');
    }
  }

  function notifyMode() {
    if (typeof modeChangeCb === 'function') {
      try {
        modeChangeCb(interactMode, {
          initialPose: initialPose,
          selectedWpIdx: selectedWpIdx,
          waypoints: waypoints,
        });
      } catch (e) { /* ignore */ }
    }
  }

  function notifyYaw() {
    if (typeof yawChangeCb === 'function') {
      try {
        yawChangeCb(getActiveYaw());
      } catch (e) { /* ignore */ }
    }
  }

  function getActiveYaw() {
    if (interactMode === 'initial_pose' && initialPose) return initialPose.yaw || 0;
    if (interactMode === 'edit_wp' && selectedWpIdx != null && waypoints[selectedWpIdx]) {
      return Number(waypoints[selectedWpIdx].yaw) || 0;
    }
    if (goalPose) return Number(goalPose.yaw) || 0;
    return 0;
  }

  function setActiveYaw(yawRad) {
    const y = Number(yawRad) || 0;
    if (interactMode === 'initial_pose') {
      if (!initialPose) return;
      initialPose.yaw = y;
    } else if (interactMode === 'edit_wp' && selectedWpIdx != null && waypoints[selectedWpIdx]) {
      waypoints[selectedWpIdx].yaw = y;
    } else if (goalPose) {
      goalPose.yaw = y;
    }
    drawOverlay();
    notifyYaw();
  }

  function setInteractMode(mode) {
    const next = mode || 'view';
    interactMode = next;
    dragState = null;
    if (next === 'initial_pose') {
      if (!initialPose) {
        const seed = getRobotTf() || goalPose || mapCenterWorld();
        initialPose = {
          x: seed ? Number(seed.x) : 0,
          y: seed ? Number(seed.y) : 0,
          yaw: seed && typeof seed.yaw === 'number' ? seed.yaw : 0,
        };
      }
    }
    if (next === 'initial_pose') selectedWpIdx = null;
    if (overlayCanvas) {
      overlayCanvas.style.cursor =
        next === 'edit_wp' || next === 'initial_pose'
          ? 'grab'
          : interactive
            ? 'default'
            : '';
    }
    drawOverlay();
    notifyMode();
    notifyYaw();
  }

  function mapCenterWorld() {
    if (!latestMap || !latestMap.info) return null;
    const info = latestMap.info;
    return {
      x: info.origin.position.x + (info.width * info.resolution) / 2,
      y: info.origin.position.y + (info.height * info.resolution) / 2,
      yaw: 0,
    };
  }

  function findWaypointHit(clientX, clientY, view) {
    const hitR = Math.max(22, view.scale * 4.5);
    for (let i = waypoints.length - 1; i >= 0; i--) {
      const wp = waypoints[i];
      if (!wp) continue;
      if (pixelDist(clientX, clientY, wp.x, wp.y, view) <= hitR) return i;
    }
    return null;
  }

  function onPointerDown(ev) {
    if (!interactive || !latestMap) return;
    const view = getViewTransform();
    if (!view) return;
    const world = canvasToWorld(ev.clientX, ev.clientY);
    if (!world) return;

    if (interactMode === 'initial_pose') {
      ev.preventDefault();
      if (initialPose && pixelDist(ev.clientX, ev.clientY, initialPose.x, initialPose.y, view) <= 18) {
        dragState = { type: 'pose' };
      } else {
        initialPose = { x: world.x, y: world.y, yaw: initialPose ? initialPose.yaw : 0 };
        dragState = { type: 'pose' };
        notifyYaw();
      }
      drawOverlay();
      notifyMode();
      return;
    }

    if (interactMode === 'edit_wp') {
      ev.preventDefault();
      const hit = findWaypointHit(ev.clientX, ev.clientY, view);
      if (hit != null) {
        selectedWpIdx = hit;
        dragState = { type: 'wp', idx: hit };
        notifyMode();
        notifyYaw();
        drawOverlay();
        return;
      }
      // add new waypoint
      if (!isClearanceOk(world.x, world.y, OBSTACLE_CLEARANCE_M)) {
        setStatus('坏点：距障碍物不足 0.3m，不能打点');
        return;
      }
      const wp = {
        name: `wp_${waypoints.length + 1}`,
        x: world.x,
        y: world.y,
        yaw: Math.PI / 2,
        bad: false,
      };
      waypoints.push(wp);
      selectedWpIdx = waypoints.length - 1;
      dragState = { type: 'wp', idx: selectedWpIdx };
      setStatus(`已添加航点 ${wp.name}`);
      notifyMode();
      notifyYaw();
      drawOverlay();
      return;
    }

    // view / goal: click existing waypoint to navigate; empty click only in goal mode
    if (interactMode === 'view' || interactMode === 'goal') {
      const hit = findWaypointHit(ev.clientX, ev.clientY, view);
      if (hit != null) {
        selectedWpIdx = hit;
        notifyMode();
        drawOverlay();
        if (typeof waypointClickCb === 'function') {
          waypointClickCb(waypoints[hit], hit, ev);
        }
        return;
      }
      if (interactMode === 'goal' && typeof clickCb === 'function') {
        clickCb(world, ev);
      }
    }
  }

  function onPointerMove(ev) {
    if (!dragState || !latestMap) return;
    const world = canvasToWorld(ev.clientX, ev.clientY);
    if (!world) return;
    if (dragState.type === 'pose' && initialPose) {
      initialPose.x = world.x;
      initialPose.y = world.y;
      drawOverlay();
    } else if (dragState.type === 'wp' && dragState.idx != null && waypoints[dragState.idx]) {
      const wp = waypoints[dragState.idx];
      wp.x = world.x;
      wp.y = world.y;
      wp.bad = !isClearanceOk(wp.x, wp.y, OBSTACLE_CLEARANCE_M);
      drawOverlay();
    }
  }

  function onPointerUp(ev) {
    if (!dragState) return;
    if (dragState.type === 'wp' && dragState.idx != null && waypoints[dragState.idx]) {
      const wp = waypoints[dragState.idx];
      wp.bad = !isClearanceOk(wp.x, wp.y, OBSTACLE_CLEARANCE_M);
      if (wp.bad) setStatus(`航点 ${wp.name || dragState.idx + 1} 为坏点（<0.3m 障碍）`);
      notifyMode();
    }
    if (dragState.type === 'pose') notifyMode();
    dragState = null;
  }

  function setInteractive(on, onClick) {
    interactive = !!on;
    clickCb = typeof onClick === 'function' ? onClick : clickCb;
    if (overlayCanvas) {
      overlayCanvas.style.pointerEvents = interactive ? 'auto' : 'none';
      overlayCanvas.style.cursor = interactive ? 'crosshair' : '';
      if (interactive && !pointerBound) {
        overlayCanvas.addEventListener('pointerdown', onPointerDown);
        window.addEventListener('pointermove', onPointerMove);
        window.addEventListener('pointerup', onPointerUp);
        pointerBound = true;
      }
    }
    if (containerEl) {
      containerEl.classList.toggle('nav-interactive', interactive);
    }
  }

  function setGoal(pose) {
    if (!pose || typeof pose.x !== 'number') {
      goalPose = null;
    } else {
      goalPose = {
        x: Number(pose.x),
        y: Number(pose.y),
        yaw: typeof pose.yaw === 'number' ? pose.yaw : 0,
      };
    }
    drawOverlay();
  }

  function clearGoal() {
    goalPose = null;
    drawOverlay();
  }

  function setStaging(pose) {
    if (!pose || typeof pose.x !== 'number') {
      stagingPose = null;
    } else {
      stagingPose = {
        x: Number(pose.x),
        y: Number(pose.y),
        yaw: typeof pose.yaw === 'number' ? pose.yaw : 0,
      };
    }
    drawOverlay();
  }

  function setWaypoints(list) {
    waypoints = annotateWaypointBadness(Array.isArray(list) ? list : []);
    drawOverlay();
  }

  function getWaypoints() {
    return annotateWaypointBadness(waypoints);
  }

  function getInitialPose() {
    return initialPose ? Object.assign({}, initialPose) : null;
  }

  function setInitialPose(pose) {
    if (!pose || typeof pose.x !== 'number') {
      initialPose = null;
    } else {
      initialPose = {
        x: Number(pose.x),
        y: Number(pose.y),
        yaw: typeof pose.yaw === 'number' ? pose.yaw : 0,
      };
    }
    drawOverlay();
    notifyMode();
    notifyYaw();
  }

  function clearInitialPose() {
    initialPose = null;
    drawOverlay();
  }

  function getSelectedWaypointIndex() {
    return selectedWpIdx;
  }

  function setSelectedWaypointIndex(idx) {
    selectedWpIdx = idx == null ? null : Number(idx);
    drawOverlay();
    notifyMode();
    notifyYaw();
  }

  function deleteSelectedWaypoint() {
    if (selectedWpIdx == null || !waypoints[selectedWpIdx]) return false;
    return deleteWaypointsByIndices([selectedWpIdx]);
  }

  /** Delete waypoints at the given indices (any order). Returns count removed. */
  function deleteWaypointsByIndices(indices) {
    const uniq = Array.from(
      new Set(
        (indices || [])
          .map((i) => Number(i))
          .filter((i) => Number.isInteger(i) && i >= 0 && i < waypoints.length),
      ),
    ).sort((a, b) => b - a);
    if (!uniq.length) return 0;
    uniq.forEach((i) => waypoints.splice(i, 1));
    if (selectedWpIdx != null) {
      if (uniq.includes(selectedWpIdx)) {
        selectedWpIdx = null;
      } else {
        const removedBefore = uniq.filter((i) => i < selectedWpIdx).length;
        selectedWpIdx -= removedBefore;
      }
    }
    waypoints = annotateWaypointBadness(waypoints);
    drawOverlay();
    notifyMode();
    return uniq.length;
  }

  function clearPlan() {
    latestPlan = null;
    latestLocalPlan = null;
    drawOverlay();
  }

  function setShowSensorFrames(on) {
    showSensorFrames = !!on;
    drawOverlay();
  }

  function getScanStatus() {
    const age = lastScanTs ? Date.now() - lastScanTs : null;
    return {
      hasScan: !!latestScan,
      ageMs: age,
      live: age != null && age < 2000,
      ranges: latestScan && latestScan.ranges ? latestScan.ranges.length : 0,
    };
  }

  function getRobotPose() {
    return getRobotTf() || (initialPose ? Object.assign({}, initialPose) : null);
  }

  function hasMap() {
    return !!latestMap;
  }

  function getObstacleClearanceM() {
    return OBSTACLE_CLEARANCE_M;
  }

  function subscribeAll() {
    if (!ros || typeof ROSLIB === 'undefined') return;

    resubscribeMapTopic();

    scanTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/scan',
      messageType: 'sensor_msgs/msg/LaserScan',
    });
    scanTopic.subscribe((msg) => {
      if (!msg || !msg.header) return;
      latestScan = msg;
      lastScanTs = Date.now();
    });

    planTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/plan',
      messageType: 'nav_msgs/msg/Path',
    });
    planTopic.subscribe((msg) => {
      const normalized = normalizePathMessage(msg);
      if (!normalized) return;
      // Keep last good global plan; empty Path is common during replan/abort.
      if (normalized.poses.length >= 2) {
        latestPlan = normalized;
      }
    });

    localPlanTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/local_plan',
      messageType: 'nav_msgs/msg/Path',
    });
    localPlanTopic.subscribe((msg) => {
      const normalized = normalizePathMessage(msg);
      if (!normalized) return;
      if (normalized.poses.length >= 2) {
        latestLocalPlan = normalized;
      }
    });

    tfTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/tf',
      messageType: 'tf2_msgs/msg/TFMessage',
    });
    tfTopic.subscribe(updateTfTree);

    tfStaticTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/tf_static',
      messageType: 'tf2_msgs/msg/TFMessage',
    });
    tfStaticTopic.subscribe(updateTfTree);
  }

  function start(options) {
    options = options || {};
    const mapId = options.mapCanvasId || 'map-canvas';
    const overlayId = options.overlayCanvasId || 'overlay-canvas';
    const containerId = options.containerId || 'map-container';
    statusCb = options.onStatus || null;
    modeChangeCb = typeof options.onModeChange === 'function' ? options.onModeChange : null;
    yawChangeCb = typeof options.onYawChange === 'function' ? options.onYawChange : null;
    preferLiveMap = options.preferLiveMap !== false;
    showSensorFrames = !!options.showSensorFrames;
    if (typeof options.onMapClick === 'function') {
      clickCb = options.onMapClick;
    }
    if (typeof options.onWaypointClick === 'function') {
      waypointClickCb = options.onWaypointClick;
    }

    mapCanvas = document.getElementById(mapId);
    overlayCanvas = document.getElementById(overlayId);
    containerEl = document.getElementById(containerId);
    if (!mapCanvas || !overlayCanvas || !containerEl) {
      console.error('[XwMapCanvas] missing canvas/container elements');
      return null;
    }
    mapCtx = mapCanvas.getContext('2d');
    overlayCtx = overlayCanvas.getContext('2d');
    resizeCanvases();

    if (options.interactive) {
      setInteractive(true, clickCb);
    }

    if (typeof ResizeObserver !== 'undefined') {
      if (resizeObserver) resizeObserver.disconnect();
      resizeObserver = new ResizeObserver(() => onContainerResize());
      resizeObserver.observe(containerEl);
    }
    window.addEventListener('resize', onContainerResize);

    if (started && ros) {
      redraw();
      return ros;
    }

    if (typeof createManagedRosConnection !== 'function' || typeof ROSLIB === 'undefined') {
      setStatus('Foxglove 脚本未加载');
      console.error('[XwMapCanvas] vendor Foxglove scripts missing');
      return null;
    }

    const url =
      typeof getFoxgloveWsUrl === 'function'
        ? getFoxgloveWsUrl()
        : `ws://${location.hostname || '127.0.0.1'}:8765`;

    const connName = options.connectionName || 'xw-map-canvas';
    ros = createManagedRosConnection({
      url: url,
      name: connName,
      onConnection: () => {
        setStatus('Foxglove 已连接 · 订阅 /map /scan /plan /local_plan /tf');
        subscribeAll();
        // If page restored into an active nav session, preferLiveMap may already be on
        // but the first latched /map was ignored while static preview loaded.
        if (preferLiveMap) {
          resubscribeMapTopic();
        }
      },
      onError: () => setStatus('Foxglove 连接错误'),
      onClose: () => setStatus('Foxglove 已断开，重连中…'),
    });

    if (!overlayTimer) {
      overlayTimer = setInterval(() => {
        if (latestMap) drawOverlay();
        scanAgeMs = lastScanTs ? Date.now() - lastScanTs : 0;
      }, 200);
    }

    if (!tfCleanTimer) {
      tfCleanTimer = setInterval(() => {
        const cutoff = Date.now() * 1e6;
        Object.keys(tfTree).forEach((key) => {
          if (cutoff - tfTree[key].stamp > TF_MAX_AGE_NS) delete tfTree[key];
        });
      }, 5000);
    }

    started = true;
    setStatus('正在连接 Foxglove…');
    return ros;
  }

  global.XwMapCanvas = {
    start: start,
    redraw: redraw,
    resizeCanvases: resizeCanvases,
    loadStaticMap: loadStaticMap,
    enableLiveMap: enableLiveMap,
    setInteractive: setInteractive,
    setInteractMode: setInteractMode,
    getInteractMode: function () {
      return interactMode;
    },
    setGoal: setGoal,
    clearGoal: clearGoal,
    setStaging: setStaging,
    setWaypoints: setWaypoints,
    getWaypoints: getWaypoints,
    setInitialPose: setInitialPose,
    getInitialPose: getInitialPose,
    clearInitialPose: clearInitialPose,
    getSelectedWaypointIndex: getSelectedWaypointIndex,
    setSelectedWaypointIndex: setSelectedWaypointIndex,
    deleteSelectedWaypoint: deleteSelectedWaypoint,
    deleteWaypointsByIndices: deleteWaypointsByIndices,
    clearPlan: clearPlan,
    setActiveYaw: setActiveYaw,
    getActiveYaw: getActiveYaw,
    isClearanceOk: isClearanceOk,
    getObstacleClearanceM: getObstacleClearanceM,
    setShowSensorFrames: setShowSensorFrames,
    getScanStatus: getScanStatus,
    getRobotPose: getRobotPose,
    hasMap: hasMap,
    hasLiveMap: hasLiveMap,
    canvasToWorld: canvasToWorld,
    DEFAULT_SENSOR_FRAMES: DEFAULT_SENSOR_FRAMES,
    LASER_DISPLAY_YAW_OFFSET: LASER_DISPLAY_YAW_OFFSET,
    OBSTACLE_CLEARANCE_M: OBSTACLE_CLEARANCE_M,
  };
})(window);
