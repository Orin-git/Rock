// Ensure header status chips + active nav on every page

function ensureStatusChips() {
  const top = document.querySelector('header.top');
  if (!top) return;

  let status = top.querySelector('.top-status');
  if (!status) {
    status = document.createElement('div');
    status.className = 'top-status';
    top.appendChild(status);
  }

  // Static LAN URL (no poll / no API) — visible when browsing via SSH tunnel
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

  const lan = document.getElementById('lanUrlBadge');
  const conn = document.getElementById('conn');
  const domain = document.getElementById('domainBadge');
  const svc = document.getElementById('svcBadge');
  if (lan) status.appendChild(lan);
  if (domain) status.appendChild(domain);
  if (svc) status.appendChild(svc);
  if (conn) status.appendChild(conn);
}

ensureStatusChips();

// Remove legacy camera nav if present (preview moved to Foxglove Desktop).
document.querySelectorAll('header.top nav a[href*="camera.html"]').forEach((a) => a.remove());

const path = location.pathname;
document.querySelectorAll('nav a').forEach((a) => {
  const href = a.getAttribute('href') || '';
  const active =
    href === path ||
    (path === '/' && href === '/') ||
    (path.endsWith('index.html') && href === '/');
  a.classList.toggle('active', !!active);
});
