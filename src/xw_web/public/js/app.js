// SPA shell — header status chips, desktop pet, nav highlight (runs once).

(function loadHudFx() {
  if (document.getElementById('hud-fx-script')) return;
  if (!document.querySelector('script[src*="hud_fx.js"]')) {
    const s = document.createElement('script');
    s.id = 'hud-fx-script';
    s.src = '/js/hud_fx.js?v=20260819-hud1';
    s.async = false;
    document.head.appendChild(s);
  }
})();

function bootDesktopPet() {
  if (document.getElementById('desktop-pet') || document.getElementById('desktop-pet-script')) {
    return;
  }
  if (!document.querySelector('link[href*="desktop_pet.css"]')) {
    const link = document.createElement('link');
    link.rel = 'stylesheet';
    link.href = '/css/desktop_pet.css?v=20260820-pet21';
    document.head.appendChild(link);
  }
  const script = document.createElement('script');
  script.id = 'desktop-pet-script';
  script.src = '/js/desktop_pet.js?v=20260820-pet21';
  script.async = true;
  document.head.appendChild(script);
}

function ensureStatusChips() {
  const top = document.querySelector('header.top');
  if (!top) return;

  let status = top.querySelector('.top-status');
  if (!status) {
    status = document.createElement('div');
    status.className = 'top-status';
    top.appendChild(status);
  }

  if (!document.getElementById('lanUrlBadge')) {
    const u = document.createElement('a');
    u.id = 'lanUrlBadge';
    u.className = 'pill lan-url';
    u.href = 'http://192.168.0.189:9000/';
    u.target = '_blank';
    u.rel = 'noopener';
    u.textContent = 'http://192.168.0.189:9000/';
    u.title = '外场本机访问地址';
    status.appendChild(u);
  }
  if (!document.getElementById('domainBadge')) {
    const d = document.createElement('div');
    d.id = 'domainBadge';
    d.className = 'pill domain-badge off';
    d.textContent = 'DOMAIN —';
    status.appendChild(d);
  }
  if (!document.getElementById('svcBadge')) {
    const s = document.createElement('div');
    s.id = 'svcBadge';
    s.className = 'pill off';
    s.textContent = '服务…';
    status.appendChild(s);
  }
  if (!document.getElementById('conn')) {
    const c = document.createElement('div');
    c.id = 'conn';
    c.className = 'pill off';
    c.textContent = '链路…';
    status.appendChild(c);
  }

  const lan = document.getElementById('lanUrlBadge');
  const conn = document.getElementById('conn');
  const domain = document.getElementById('domainBadge');
  const svc = document.getElementById('svcBadge');
  if (lan) status.appendChild(lan);
  if (domain) status.appendChild(domain);
  if (svc) status.appendChild(svc);
  if (conn) status.appendChild(conn);
}

export function setActiveNav(path) {
  const p = path === '/index.html' ? '/' : path;
  document.querySelectorAll('header.top nav a').forEach((a) => {
    const href = a.getAttribute('href') || '';
    const hrefPath = href.split('?')[0];
    const active =
      hrefPath === p ||
      (p === '/' && hrefPath === '/') ||
      (p.endsWith('index.html') && hrefPath === '/');
    a.classList.toggle('active', !!active);
  });
}

export function initShell() {
  bootDesktopPet();
  ensureStatusChips();
  document.querySelectorAll('header.top nav a[href*="camera.html"]').forEach((a) => a.remove());
  setActiveNav(location.pathname);
}
