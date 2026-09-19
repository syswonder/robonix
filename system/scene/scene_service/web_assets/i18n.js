
// One language on screen, chosen once and remembered. Both the shell and the
// view in its iframe read this key, so the choice crosses the frame boundary
// without a server round trip.
const I18N = __TABLE__;
const LANGS = ['en', 'zh'];
function langGet() {
  try {
    const v = localStorage.getItem('sceneLang');
    if (LANGS.includes(v)) return v;
  } catch (_) {}
  // First visit follows the browser rather than assuming English.
  return (navigator.language || '').toLowerCase().startsWith('zh') ? 'zh' : 'en';
}
function t(key, lang) {
  const row = I18N[key];
  // A missing key shows its key rather than blank: a gap in the table should
  // be visible in a screenshot, not silently render an empty control.
  return row ? (row[lang || langGet()] || row.en || key) : key;
}
function applyLang(lang, root) {
  const d = root || document;
  d.documentElement && (d.documentElement.lang = lang === 'zh' ? 'zh' : 'en');
  d.querySelectorAll('[data-i18n]').forEach(
    el => { el.textContent = t(el.dataset.i18n, lang); });
  d.querySelectorAll('[data-i18n-title]').forEach(
    el => { el.title = t(el.dataset.i18nTitle, lang); });
  d.querySelectorAll('[data-i18n-ph]').forEach(
    el => { el.placeholder = t(el.dataset.i18nPh, lang); });
  d.querySelectorAll('[data-i18n-aria]').forEach(
    el => { el.setAttribute('aria-label', t(el.dataset.i18nAria, lang)); });
}
function langSet(lang) {
  try { localStorage.setItem('sceneLang', lang); } catch (_) {}
  applyLang(lang);
  // The views are iframes with their own documents; tell them rather than
  // waiting for the next navigation.
  document.querySelectorAll('iframe').forEach(f => {
    try { f.contentWindow.postMessage({sceneLang: lang}, '*'); } catch (_) {}
  });
}
// A frame applies what it is told, and what it already had on load.
window.addEventListener('message', e => {
  const lang = e.data && e.data.sceneLang;
  if (LANGS.includes(lang)) {
    try { localStorage.setItem('sceneLang', lang); } catch (_) {}
    applyLang(lang);
  }
});
