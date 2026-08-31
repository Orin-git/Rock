/** SPA navigation helper — set by spa.js at boot. */
let _go = null;

export function setNavigator(fn) {
  _go = fn;
}

export function navigateTo(path, query = '') {
  if (_go) {
    _go(path, query);
    return true;
  }
  const qs = query ? (query.startsWith('?') ? query : `?${query}`) : '';
  location.href = path + qs;
  return false;
}
