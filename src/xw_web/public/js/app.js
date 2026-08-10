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
  if (!document.getElementById('domainBadge')) {
    const d = document.createElement('div');
    d.id = 'domainBadge';
    d.className = 'pill domain-badge off';
    d.textContent = 'DOMAIN —';
    top.appendChild(d);
  }
  if (!document.getElementById('svcBadge')) {
    const s = document.createElement('div');
    s.id = 'svcBadge';
    s.className = 'pill off';
    s.textContent = '服务…';
    top.appendChild(s);
  }
  // ensure conn is last visual group — insert chips before conn if present
  const conn = document.getElementById('conn');
  const domain = document.getElementById('domainBadge');
  const svc = document.getElementById('svcBadge');
  if (conn && domain && svc) {
    top.insertBefore(domain, conn);
    top.insertBefore(svc, conn);
  }
}

ensureStatusChips();
