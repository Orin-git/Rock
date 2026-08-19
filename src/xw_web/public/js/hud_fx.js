/**
 * Xiaowei Gen2 — lightweight HUD FX layer
 * Modes: high | low | off  (localStorage key: xw_fx_mode)
 * - high: particles (dashboard) + map scanline + pill breathe
 * - low: chrome only, no particles / scanline
 * - off: minimal visuals
 */
(function (global) {
  'use strict';

  const KEY = 'xw_fx_mode';
  const MODES = ['high', 'low', 'off'];

  function getMode() {
    try {
      const m = localStorage.getItem(KEY);
      if (MODES.includes(m)) return m;
    } catch (_) { /* ignore */ }
    // Default: FX off (user preference — visual modes were hard to distinguish)
    return 'off';
  }

  function setMode(mode) {
    const m = MODES.includes(mode) ? mode : 'off';
    try { localStorage.setItem(KEY, m); } catch (_) { /* ignore */ }
    applyMode(m);
    return m;
  }

  function cycleMode() {
    // Keep API but no longer cycle in UI — always off
    return setMode('off');
  }

  function applyMode(mode) {
    const root = document.documentElement;
    root.classList.remove('fx-high', 'fx-low', 'fx-off');
    root.classList.add('fx-' + mode);
    root.dataset.fx = mode;
    const btn = document.getElementById('fxToggle');
    if (btn) btn.remove();
    enhanceMapContainers();
  }

  function enhanceMapContainers() {
    document.querySelectorAll('#map-container').forEach((el) => {
      el.classList.add('map-hud');
      if (document.documentElement.dataset.fx === 'high' && !el.querySelector('.map-scanline')) {
        const scan = document.createElement('div');
        scan.className = 'map-scanline';
        scan.setAttribute('aria-hidden', 'true');
        scan.innerHTML = '<i></i>';
        el.appendChild(scan);
      }
      if (document.documentElement.dataset.fx !== 'high') {
        el.querySelectorAll('.map-scanline').forEach((n) => n.remove());
      }
    });
  }

  /** Lightweight particle field — only when mount requested and mode=high */
  function startParticles(host) {
    if (!host || document.documentElement.dataset.fx !== 'high') return null;
    if (document.getElementById('hud-particles')) return null;

    const canvas = document.createElement('canvas');
    canvas.id = 'hud-particles';
    canvas.setAttribute('aria-hidden', 'true');
    (host === document.body ? document.body : host).appendChild(canvas);
    if (host === document.body) {
      document.body.insertBefore(canvas, document.body.firstChild);
    }

    const ctx = canvas.getContext('2d', { alpha: true });
    let w = 0;
    let h = 0;
    let raf = 0;
    let last = 0;
    const COUNT = Math.min(56, Math.floor((global.innerWidth * global.innerHeight) / 28000));
    const particles = [];

    function resize() {
      w = canvas.width = global.innerWidth;
      h = canvas.height = global.innerHeight;
    }

    function spawn() {
      particles.length = 0;
      for (let i = 0; i < COUNT; i++) {
        particles.push({
          x: Math.random() * w,
          y: Math.random() * h,
          vx: (Math.random() - 0.5) * 0.22,
          vy: -0.12 - Math.random() * 0.28,
          r: 0.6 + Math.random() * 1.4,
          a: 0.15 + Math.random() * 0.45,
          hue: Math.random() > 0.82 ? 300 : 185,
        });
      }
    }

    function tick(t) {
      if (document.documentElement.dataset.fx !== 'high') {
        cancelAnimationFrame(raf);
        canvas.remove();
        return;
      }
      const dt = Math.min(32, t - last || 16);
      last = t;
      ctx.clearRect(0, 0, w, h);
      for (let i = 0; i < particles.length; i++) {
        const p = particles[i];
        p.x += p.vx * (dt / 16);
        p.y += p.vy * (dt / 16);
        if (p.y < -4) { p.y = h + 4; p.x = Math.random() * w; }
        if (p.x < -4) p.x = w + 4;
        if (p.x > w + 4) p.x = -4;
        ctx.beginPath();
        ctx.arc(p.x, p.y, p.r, 0, Math.PI * 2);
        ctx.fillStyle = `hsla(${p.hue}, 55%, 42%, ${p.a * 0.55})`;
        ctx.fill();
      }
      // sparse links — every other frame skip for CPU
      if ((t / 16 | 0) % 2 === 0) {
        ctx.lineWidth = 0.55;
        for (let i = 0; i < particles.length; i++) {
          const a = particles[i];
          for (let j = i + 1; j < Math.min(i + 4, particles.length); j++) {
            const b = particles[j];
            const dx = a.x - b.x;
            const dy = a.y - b.y;
            const d2 = dx * dx + dy * dy;
            if (d2 < 90 * 90) {
              ctx.strokeStyle = `rgba(12, 127, 150, ${0.1 * (1 - d2 / 8100)})`;
              ctx.beginPath();
              ctx.moveTo(a.x, a.y);
              ctx.lineTo(b.x, b.y);
              ctx.stroke();
            }
          }
        }
      }
      raf = requestAnimationFrame(tick);
    }

    resize();
    spawn();
    global.addEventListener('resize', () => { resize(); spawn(); }, { passive: true });
    raf = requestAnimationFrame(tick);

    return {
      stop() {
        cancelAnimationFrame(raf);
        canvas.remove();
      },
    };
  }

  function ensureFxToggle() {
    // FX UI removed — always off
    const btn = document.getElementById('fxToggle');
    if (btn) btn.remove();
  }

  function ensureDashNav() {
    // 态势已并入总览：清掉残留入口
    document.querySelectorAll('header.top nav a[href="/pages/dashboard.html"]').forEach((a) => a.remove());
  }

  function boot() {
    // Ensure HUD stylesheet is present on every page
    if (!document.querySelector('link[href*="hud.css"]')) {
      const link = document.createElement('link');
      link.rel = 'stylesheet';
      link.href = '/css/hud.css?v=20260819-hud3';
      document.head.appendChild(link);
    }
    try { localStorage.setItem(KEY, 'off'); } catch (_) { /* ignore */ }
    ensureDashNav();
    ensureFxToggle();
    applyMode('off');
    // Re-enhance maps that mount late
    if (global.MutationObserver) {
      let scheduled = false;
      const mo = new MutationObserver(() => {
        if (scheduled) return;
        scheduled = true;
        requestAnimationFrame(() => {
          scheduled = false;
          enhanceMapContainers();
        });
      });
      mo.observe(document.documentElement, { childList: true, subtree: true });
    }
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot);
  } else {
    boot();
  }

  global.XwHudFx = {
    getMode,
    setMode,
    cycleMode,
    startParticles,
    enhanceMapContainers,
  };
})(window);
