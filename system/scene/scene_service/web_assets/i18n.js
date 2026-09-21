
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
// A sentence with `{name}` holes in it, filled from `vars`. An unknown
// hole is left as written rather than blanked, so a table row that has
// drifted from its call site shows up on screen instead of going quiet.
function tv(key, vars, lang) {
  return t(key, lang).replace(/\{(\w+)\}/g,
    (whole, name) => (vars && name in vars) ? String(vars[name]) : whole);
}
function applyLang(lang, root) {
  const d = root || document;
  d.documentElement && (d.documentElement.lang = lang === 'zh' ? 'zh' : 'en');
  d.querySelectorAll('[data-i18n]').forEach(el => {
    // The values travel with the key: re-rendering a sentence in the other
    // language means filling its holes again, not reusing the old text.
    const packed = el.dataset.i18nVars;
    let vars = null;
    if (packed) { try { vars = JSON.parse(packed); } catch (_) {} }
    el.textContent = vars ? tv(el.dataset.i18n, vars, lang)
                          : t(el.dataset.i18n, lang);
  });
  // The language switch offers the language you are not in, so it is
  // labelled from the other table. Keeping each `strings_<lang>.json` to
  // its own language is the point: a translator opens one file and sees
  // only the language they speak.
  d.querySelectorAll('[data-i18n-other]').forEach(el => {
    el.textContent = t(el.dataset.i18nOther, lang === 'zh' ? 'en' : 'zh');
  });
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
