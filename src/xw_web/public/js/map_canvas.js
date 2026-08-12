/**
 * Gen2 map canvas — Foxglove WS draw /map + /scan + robot pose.
 * Also used by navigation: click-to-goal, waypoints, static PGM preview.
 * Depends on vendor: foxglove_bundle.js, roslib_foxglove.js, ros_ws_helper.js
 * Export: window.XwMapCanvas
 */
(function (global) {
  'use strict';

  // Web-only: rotate laser overlay on map canvas (does not affect /scan or SLAM).
  const LASER_DISPLAY_YAW_OFFSET = Math.PI;

  const GLOW_CONFIG = {
    dilateRadius: 1,
    blurPx: 0.8,
    alpha: 0.35,
  };

  /** Default sensor frames relative to base_link (from xw_gen2.urdf). */
  const DEFAULT_SENSOR_FRAMES = [
    { id: 'lidar', frame: 'lidar_link', label: 'LiDAR', color: '#c23048' },
    { id: 'camera_front', frame: 'camera_front_link', label: '深度前视', color: '#22c55e' },
    { id: 'camera_front_2', frame: 'camera_front_2_link', label: '深度前视二号', color: '#16a34a' },
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
  let tfTopic = null;
  let tfStaticTopic = null;

  let latestMap = null;
  let latestScan = null;
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
  let interactive = false;
  let preferLiveMap = true;
  let showSensorFrames = false;
  let goalPose = null; // { x, y, yaw }
  let waypoints = []; // [{ name, x, y, yaw? }]
  let scanAgeMs = 0;
  let lastScanTs = 0;
  let clickBound = false;

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
          pixels[pixelOffset] = 38;
          pixels[pixelOffset + 1] = 34;
          pixels[pixelOffset + 2] = 48;
          pixels[pixelOffset + 3] = 255;
          glowPixels[pixelOffset + 3] = 0;
        } else if (value === 0) {
          pixels[pixelOffset] = 50;
          pixels[pixelOffset + 1] = 72;
          pixels[pixelOffset + 2] = 110;
          pixels[pixelOffset + 3] = 255;
          glowPixels[pixelOffset + 3] = 0;
        } else {
          const t = value / 100;
          const wr = Math.round(55 + t * 145);
          const wg = Math.round(115 + t * 140);
          const wb = Math.round(160 + t * 95);
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
    }
    return true;
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
    mapCtx.strokeStyle = 'rgba(0, 200, 240, 0.12)';
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

  function drawPoseMarker(view, x, y, yaw, fill, stroke, radius) {
    const p = worldToPixel(x, y, view);
    const r = radius || Math.max(7, view.scale * 1.8);
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

  function drawOverlay() {
    if (!latestMap || !overlayCtx) return;
    overlayCtx.clearRect(0, 0, overlayCanvas.width, overlayCanvas.height);
    const view = getViewTransform();
    if (!view) return;

    const robotTf = getRobotTf();

    if (latestScan) {
      let tf = null;
      if (latestScan.header && latestScan.header.frame_id) {
        const fid = String(latestScan.header.frame_id).replace(/^\//, '');
        tf = getTransform('map', fid);
      }
      if (!tf) tf = robotTf;
      if (tf) {
        const robotX = tf.x;
        const robotY = tf.y;
        const scanYaw = tf.yaw + LASER_DISPLAY_YAW_OFFSET;
        overlayCtx.fillStyle = '#c23048';
        const ranges = latestScan.ranges || [];
        for (let i = 0; i < ranges.length; i++) {
          const range = ranges[i];
          if (range < latestScan.range_min || range > latestScan.range_max) continue;
          const angle = latestScan.angle_min + i * latestScan.angle_increment;
          const lx = range * Math.cos(angle);
          const ly = range * Math.sin(angle);
          const wx = robotX + Math.cos(scanYaw) * lx - Math.sin(scanYaw) * ly;
          const wy = robotY + Math.sin(scanYaw) * lx + Math.cos(scanYaw) * ly;
          const p = worldToPixel(wx, wy, view);
          overlayCtx.beginPath();
          overlayCtx.arc(p.px, p.py, Math.max(2.5, view.scale * 0.9), 0, 2 * Math.PI);
          overlayCtx.fill();
        }
      }
    }

    if (Array.isArray(waypoints)) {
      waypoints.forEach((wp, idx) => {
        if (!wp || typeof wp.x !== 'number') return;
        const p = drawPoseMarker(
          view,
          wp.x,
          wp.y,
          typeof wp.yaw === 'number' ? wp.yaw : undefined,
          'rgba(124, 58, 237, 0.85)',
          '#5b21b6',
          Math.max(6, view.scale * 1.5)
        );
        overlayCtx.fillStyle = '#fff';
        overlayCtx.font = 'bold 11px sans-serif';
        overlayCtx.textAlign = 'center';
        overlayCtx.textBaseline = 'middle';
        overlayCtx.fillText(String(idx + 1), p.px, p.py);
      });
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

    if (robotTf) {
      drawPoseMarker(
        view,
        robotTf.x,
        robotTf.y,
        robotTf.yaw,
        'rgba(0, 150, 255, 0.85)',
        '#0b6e99',
        Math.max(8, view.scale * 2)
      );
    }

    if (showSensorFrames && robotTf) {
      DEFAULT_SENSOR_FRAMES.forEach((sf) => {
        let tf = getTransform('map', sf.frame);
        if (!tf) {
          // Fall back: base_link relative offsets from URDF if TF missing
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

  function onMapMessage(message) {
    if (!preferLiveMap) return;
    try {
      if (!applyMapMessage(message, '地图已更新（/map）')) return;
    } catch (err) {
      console.error('[XwMapCanvas] /map handler error', err);
      setStatus('地图解析失败');
    }
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

  /** Load saved map from /api/map op=5 payload (pgm_b64). */
  function loadStaticMap(payload) {
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
    preferLiveMap = false;
    const msg = {
      header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'map' },
      info: {
        resolution: Number(payload.resolution) || 0.05,
        width: width,
        height: height,
        origin: {
          position: {
            x: Number(payload.origin_x) || 0,
            y: Number(payload.origin_y) || 0,
            z: 0,
          },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
      },
      data: data,
    };
    return applyMapMessage(msg, `静态地图已加载 · ${width}×${height}`);
  }

  function enableLiveMap() {
    preferLiveMap = true;
    setStatus('等待 /map 实时话题…');
  }

  function onOverlayClick(ev) {
    if (!interactive || typeof clickCb !== 'function') return;
    const world = canvasToWorld(ev.clientX, ev.clientY);
    if (!world) return;
    clickCb(world, ev);
  }

  function setInteractive(on, onClick) {
    interactive = !!on;
    clickCb = typeof onClick === 'function' ? onClick : clickCb;
    if (overlayCanvas) {
      overlayCanvas.style.pointerEvents = interactive ? 'auto' : 'none';
      overlayCanvas.style.cursor = interactive ? 'crosshair' : '';
      if (interactive && !clickBound) {
        overlayCanvas.addEventListener('click', onOverlayClick);
        clickBound = true;
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

  function setWaypoints(list) {
    waypoints = Array.isArray(list) ? list : [];
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
    return getRobotTf();
  }

  function hasMap() {
    return !!latestMap;
  }

  function subscribeAll() {
    if (!ros || typeof ROSLIB === 'undefined') return;

    mapTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/map',
      messageType: 'nav_msgs/msg/OccupancyGrid',
    });
    mapTopic.subscribe(onMapMessage);

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
    preferLiveMap = options.preferLiveMap !== false;
    showSensorFrames = !!options.showSensorFrames;
    if (typeof options.onMapClick === 'function') {
      clickCb = options.onMapClick;
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
      resizeObserver = new ResizeObserver(() => redraw());
      resizeObserver.observe(containerEl);
    }
    window.addEventListener('resize', redraw);

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
        setStatus('Foxglove 已连接 · 订阅 /map /scan /tf');
        subscribeAll();
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
    setGoal: setGoal,
    clearGoal: clearGoal,
    setWaypoints: setWaypoints,
    setShowSensorFrames: setShowSensorFrames,
    getScanStatus: getScanStatus,
    getRobotPose: getRobotPose,
    hasMap: hasMap,
    canvasToWorld: canvasToWorld,
    DEFAULT_SENSOR_FRAMES: DEFAULT_SENSOR_FRAMES,
    LASER_DISPLAY_YAW_OFFSET: LASER_DISPLAY_YAW_OFFSET,
  };
})(window);
