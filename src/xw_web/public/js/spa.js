/**
 * Xiaowei Gen2 SPA shell — persistent header, swap <main> content only.
 */
import { connect } from '/js/api.js';
import { initShell, setActiveNav } from '/js/app.js';
import { setNavigator } from '/js/navigate.js';

const VENDOR_SCRIPTS = [
  '/js/vendor/foxglove_bundle.js',
  '/js/vendor/roslib_foxglove.js',
  '/js/vendor/ros_ws_helper.js',
  '/js/map_canvas.js?v=20260902-mapflip',
];

const ROUTES = {
  '/': {
    partial: '/partials/overview.html',
    bodyClass: 'dash-body',
    mainClass: 'dash-main',
    title: '小维二代 · 总览',
    css: ['/css/hud.css'],
    module: () => import('/js/overview_page.js'),
  },
  '/index.html': {
    partial: '/partials/overview.html',
    bodyClass: 'dash-body',
    mainClass: 'dash-main',
    title: '小维二代 · 总览',
    css: ['/css/hud.css'],
    module: () => import('/js/overview_page.js'),
  },
  '/pages/topics.html': {
    partial: '/partials/topics.html',
    bodyClass: 'topics-body',
    mainClass: 'topics-page',
    title: '话题与节点 · 小维二代',
    css: [],
    module: () => import('/js/topics_page.js'),
  },
  '/pages/viz.html': {
    partial: '/partials/viz.html',
    bodyClass: 'viz-body',
    mainClass: 'viz-page',
    title: '可视化 · 小维二代',
    css: [],
    module: () => import('/js/viz_page.js'),
  },
  '/pages/teleop.html': {
    partial: '/partials/teleop.html',
    bodyClass: 'teleop-body',
    mainClass: 'teleop-main',
    title: '遥控 · 小维二代',
    css: ['/css/teleop.css?v=20260819-tele2'],
    module: () => import('/js/teleop_page.js'),
  },
  '/pages/mapping.html': {
    partial: '/partials/mapping.html',
    bodyClass: 'mapping-body',
    mainClass: 'mapping-page',
    title: '建图 · 小维二代',
    css: [],
    vendors: VENDOR_SCRIPTS,
    module: () => import('/js/mapping_page.js'),
  },
  '/pages/navigation.html': {
    partial: '/partials/navigation.html',
    bodyClass: 'nav-body',
    mainClass: 'nav-page',
    title: '导航 · 小维二代',
    css: [],
    vendors: VENDOR_SCRIPTS,
    module: () => import('/js/navigation_page.js'),
  },
  '/pages/maps.html': {
    partial: '/partials/maps.html',
    bodyClass: 'maps-body',
    mainClass: 'maps-main',
    title: '地图 · 小维二代',
    css: ['/css/maps.css?v=20260819-maps2'],
    module: () => import('/js/maps_page.js'),
  },
  '/pages/map_beautify.html': {
    partial: '/partials/map_beautify.html',
    bodyClass: 'beautify-body',
    mainClass: 'beautify-page',
    title: '美化 · 小维二代',
    css: [],
    module: () => import('/js/map_beautify_page.js'),
  },
  '/pages/settings.html': {
    partial: '/partials/settings.html',
    bodyClass: 'settings-body',
    mainClass: 'settings-page',
    title: '设置 · 小维二代',
    css: [],
    module: () => import('/js/settings_page.js'),
  },
};

const partialCache = new Map();
let currentPath = null;
let unmountPage = null;
let navigating = false;

function normalizePath(path) {
  if (!path || path === '/index.html') return '/';
  return path;
}

function routeForPath(path) {
  const p = normalizePath(path);
  return ROUTES[p] || null;
}

function ensureStylesheet(href) {
  if (document.querySelector(`link[rel="stylesheet"][href="${href}"]`)) return;
  const link = document.createElement('link');
  link.rel = 'stylesheet';
  link.href = href;
  link.dataset.spaCss = '1';
  document.head.appendChild(link);
}

function syncExtraCss(list) {
  const want = new Set(list || []);
  document.querySelectorAll('link[data-spa-css="1"]').forEach((el) => {
    if (!want.has(el.getAttribute('href'))) el.remove();
  });
  (list || []).forEach(ensureStylesheet);
}

async function loadScriptOnce(src) {
  if (document.querySelector(`script[src="${src}"]`)) return;
  await new Promise((resolve, reject) => {
    const s = document.createElement('script');
    s.src = src;
    s.async = false;
    s.onload = () => resolve();
    s.onerror = () => reject(new Error(`script load failed: ${src}`));
    document.head.appendChild(s);
  });
}

async function loadVendors(urls) {
  for (const url of urls || []) {
    await loadScriptOnce(url);
  }
}

async function fetchPartial(url) {
  if (partialCache.has(url)) return partialCache.get(url);
  const res = await fetch(url, { cache: 'no-store' });
  if (!res.ok) throw new Error(`partial ${url}: ${res.status}`);
  const html = await res.text();
  partialCache.set(url, html);
  return html;
}

function parseQuery(search) {
  const raw = (search || '').replace(/^\?/, '');
  return raw ? new URLSearchParams(raw) : new URLSearchParams();
}

export async function navigate(path, { replace = false, query = '' } = {}) {
  const routePath = normalizePath(path);
  const route = routeForPath(routePath);
  if (!route || navigating) return false;

  navigating = true;
  const outlet = document.getElementById('spa-outlet');
  if (!outlet) {
    navigating = false;
    return false;
  }

  try {
    if (typeof unmountPage === 'function') {
      try {
        unmountPage();
      } catch (e) {
        console.warn('[spa] unmount error', e);
      }
      unmountPage = null;
    }

    outlet.classList.add('spa-loading');
    document.body.className = route.bodyClass;
    document.title = route.title;
    syncExtraCss(route.css);
    setActiveNav(routePath);

    if (route.vendors) await loadVendors(route.vendors);

    const html = await fetchPartial(route.partial);
    outlet.className = route.mainClass || '';
    outlet.innerHTML = html;

    const mod = await route.module();
    if (typeof mod.mount === 'function') {
      const qs = typeof query === 'string' ? parseQuery(query) : query;
      const ret = mod.mount({ path: routePath, query: qs });
      unmountPage = typeof ret === 'function' ? ret : mod.unmount || null;
    }

    const qs =
      typeof query === 'string'
        ? query.replace(/^\?/, '')
        : query instanceof URLSearchParams
          ? query.toString()
          : '';
    const url = routePath + (qs ? `?${qs}` : '');
    const state = { path: routePath, query: qs };
    if (replace) history.replaceState(state, '', url);
    else if (routePath !== currentPath || location.search.replace(/^\?/, '') !== qs) {
      history.pushState(state, '', url);
    }

    currentPath = routePath;
    return true;
  } catch (e) {
    console.error('[spa] navigate failed', e);
    outlet.innerHTML = `<p class="muted pad">页面加载失败：${e.message || e}</p>`;
    return false;
  } finally {
    outlet.classList.remove('spa-loading');
    navigating = false;
  }
}

function onNavClick(e) {
  const a = e.target.closest('a[href]');
  if (!a || a.target === '_blank') return;
  const href = a.getAttribute('href') || '';
  if (!href.startsWith('/') || href.startsWith('//')) return;
  const [path, qs] = href.split('?');
  if (!routeForPath(path)) return;
  e.preventDefault();
  navigate(path, { query: qs || '' });
}

function boot() {
  initShell();
  connect();
  setNavigator((path, query) => {
    navigate(path, { query });
  });

  document.querySelector('header.top nav')?.addEventListener('click', onNavClick);
  document.addEventListener('click', (e) => {
    if (e.target.closest('header.top nav')) return;
    onNavClick(e);
  });

  window.addEventListener('popstate', () => {
    const path = normalizePath(location.pathname);
    navigate(path, {
      replace: true,
      query: location.search.replace(/^\?/, ''),
    });
  });

  const startPath = normalizePath(location.pathname);
  if (!routeForPath(startPath)) {
    navigate('/', { replace: true });
    return;
  }
  navigate(startPath, {
    replace: true,
    query: location.search.replace(/^\?/, ''),
  });
}

boot();
