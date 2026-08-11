// Ensure header status chips + active nav on every page
const path = location.pathname;
document.querySelectorAll('nav a').forEach((a) => {
  const href = a.getAttribute('href');
  const active =
    href === path ||
    (path === '/' && href === '/') ||
    (path.endsWith('index.html') && href === '/');
  a.classList.toggle('active', !!active);
});

function ensureStatusChips() {
  const top = document.querySelector('header.top');
  if (!top) return;

  let status = top.querySelector('.top-status');
  if (!status) {
    status = document.createElement('div');
    status.className = 'top-status';
    top.appendChild(status);
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

  const conn = document.getElementById('conn');
  const domain = document.getElementById('domainBadge');
  const svc = document.getElementById('svcBadge');
  if (conn && conn.parentElement !== status) {
    status.appendChild(conn);
  }
  if (domain && domain.parentElement !== status) {
    status.insertBefore(domain, conn || null);
  }
  if (svc && svc.parentElement !== status) {
    status.insertBefore(svc, conn || null);
  }
  // keep order: domain → svc → conn
  if (domain) status.appendChild(domain);
  if (svc) status.appendChild(svc);
  if (conn) status.appendChild(conn);
}

ensureStatusChips();
