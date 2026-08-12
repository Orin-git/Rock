/**
 * Occupancy-grid beautify helpers (PGM trinary: 0 occupied / 254 free / 205 unknown).
 * Ported from gen1 map_beautify_algo.js as ES module.
 */

export const OCCUPIED = 0;
export const FREE = 254;
export const UNKNOWN = 205;

export function isOccupied(v) {
  return v <= 50;
}

function cloneData(data) {
  return data.slice();
}

function toOccupiedMask(data) {
  const mask = new Uint8Array(data.length);
  for (let i = 0; i < data.length; i++) {
    mask[i] = isOccupied(data[i]) ? 1 : 0;
  }
  return mask;
}

function countOccupiedNeighbors(mask, width, height, x, y, radius) {
  let count = 0;
  for (let dy = -radius; dy <= radius; dy++) {
    for (let dx = -radius; dx <= radius; dx++) {
      if (dx === 0 && dy === 0) continue;
      const nx = x + dx;
      const ny = y + dy;
      if (nx >= 0 && ny >= 0 && nx < width && ny < height && mask[ny * width + nx]) {
        count++;
      }
    }
  }
  return count;
}

function erode(mask, width, height, radius) {
  const out = new Uint8Array(mask.length);
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      let keep = true;
      for (let dy = -radius; dy <= radius && keep; dy++) {
        for (let dx = -radius; dx <= radius && keep; dx++) {
          const nx = x + dx;
          const ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= width || ny >= height || !mask[ny * width + nx]) {
            keep = false;
          }
        }
      }
      out[y * width + x] = keep ? 1 : 0;
    }
  }
  return out;
}

function dilate(mask, width, height, radius) {
  const out = new Uint8Array(mask.length);
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      let hit = false;
      for (let dy = -radius; dy <= radius && !hit; dy++) {
        for (let dx = -radius; dx <= radius && !hit; dx++) {
          const nx = x + dx;
          const ny = y + dy;
          if (nx >= 0 && ny >= 0 && nx < width && ny < height && mask[ny * width + nx]) {
            hit = true;
          }
        }
      }
      out[y * width + x] = hit ? 1 : 0;
    }
  }
  return out;
}

function morphClose(mask, width, height, radius) {
  return erode(dilate(mask, width, height, radius), width, height, radius);
}

function morphOpen(mask, width, height, radius) {
  return dilate(erode(mask, width, height, radius), width, height, radius);
}

function removeIsolatedNoise(mask, width, height, minNeighbors) {
  const out = mask.slice();
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      if (!mask[y * width + x]) continue;
      if (countOccupiedNeighbors(mask, width, height, x, y, 2) < minNeighbors) {
        out[y * width + x] = 0;
      }
    }
  }
  return out;
}

function maskToData(mask, original) {
  const out = cloneData(original);
  for (let i = 0; i < mask.length; i++) {
    if (mask[i]) out[i] = OCCUPIED;
    else if (isOccupied(original[i])) out[i] = FREE;
  }
  return out;
}

function smoothBoundaryMask(mask, width, height, iterations) {
  let current = mask.slice();
  for (let iter = 0; iter < iterations; iter++) {
    const next = current.slice();
    for (let y = 1; y < height - 1; y++) {
      for (let x = 1; x < width - 1; x++) {
        let count = 0;
        for (let dy = -1; dy <= 1; dy++) {
          for (let dx = -1; dx <= 1; dx++) {
            if (current[(y + dy) * width + (x + dx)]) count++;
          }
        }
        const idx = y * width + x;
        if (current[idx] && count >= 5) next[idx] = 1;
        else if (!current[idx] && count >= 6) next[idx] = 1;
        else if (current[idx] && count <= 2) next[idx] = 0;
        else next[idx] = count >= 5 ? 1 : 0;
      }
    }
    current = next;
  }
  return current;
}

/** Connect nearby fragments, fill small gaps, remove speckles; protect real obstacles. */
export function lightBeautify(data, width, height) {
  const original = toOccupiedMask(data);
  const protectedCore = dilate(original, width, height, 1);

  let mask = original.slice();
  mask = removeIsolatedNoise(mask, width, height, 2);
  mask = morphClose(mask, width, height, 2);
  mask = removeIsolatedNoise(mask, width, height, 3);
  mask = morphClose(mask, width, height, 1);
  mask = smoothBoundaryMask(mask, width, height, 1);

  for (let i = 0; i < mask.length; i++) {
    if (protectedCore[i]) mask[i] = 1;
  }
  return maskToData(mask, data);
}

/** Stronger wall smooth on top of lightBeautify. */
export function wallSmooth(data, width, height) {
  const original = toOccupiedMask(data);
  const protectedCore = dilate(original, width, height, 2);

  let base = lightBeautify(data, width, height);
  let mask = toOccupiedMask(base);
  mask = morphClose(mask, width, height, 2);
  mask = smoothBoundaryMask(mask, width, height, 3);
  mask = morphOpen(mask, width, height, 1);
  mask = morphClose(mask, width, height, 1);
  mask = smoothBoundaryMask(mask, width, height, 2);

  for (let i = 0; i < mask.length; i++) {
    if (protectedCore[i]) mask[i] = 1;
  }
  return maskToData(mask, base);
}

export function bresenham(x0, y0, x1, y1, callback) {
  let dx = Math.abs(x1 - x0);
  let dy = Math.abs(y1 - y0);
  const sx = x0 < x1 ? 1 : -1;
  const sy = y0 < y1 ? 1 : -1;
  let err = dx - dy;
  let x = x0;
  let y = y0;
  for (;;) {
    callback(x, y);
    if (x === x1 && y === y1) break;
    const e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }
}

export function rasterizeThickLine(data, width, height, x0, y0, x1, y1, value, radius) {
  bresenham(x0, y0, x1, y1, (cx, cy) => {
    for (let dy = -radius; dy <= radius; dy++) {
      for (let dx = -radius; dx <= radius; dx++) {
        if (dx * dx + dy * dy > radius * radius) continue;
        const x = cx + dx;
        const y = cy + dy;
        if (x >= 0 && y >= 0 && x < width && y < height) {
          data[y * width + x] = value;
        }
      }
    }
  });
}

export function stampBrush(data, width, height, cx, cy, value, radius) {
  const r = Math.max(0, Math.floor(radius));
  const rSq = r * r;
  for (let dy = -r; dy <= r; dy++) {
    for (let dx = -r; dx <= r; dx++) {
      if (dx * dx + dy * dy > rSq) continue;
      const x = cx + dx;
      const y = cy + dy;
      if (x >= 0 && y >= 0 && x < width && y < height) {
        data[y * width + x] = value;
      }
    }
  }
}
