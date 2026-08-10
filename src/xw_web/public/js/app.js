const path = location.pathname;
document.querySelectorAll('nav a').forEach((a) => {
  const href = a.getAttribute('href');
  const active =
    href === path ||
    (path === '/' && href === '/') ||
    (path.endsWith('index.html') && href === '/');
  a.classList.toggle('active', !!active);
});
