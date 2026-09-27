
// The page language, stored under one key that the shell and its frames share.
const I18N = __TABLE__;
const LANGS = ['en', 'zh'];
function langGet() {
  try {
    const v = localStorage.getItem('sceneLang');
    if (LANGS.includes(v)) return v;
  } catch (_) {}
  return (navigator.language || '').toLowerCase().startsWith('zh') ? 'zh' : 'en';
}
// A missing key renders as the key, so a gap shows on screen.
function t(key, lang) {
  const row = I18N[key];
  return row ? (row[lang || langGet()] || row.en || key) : key;
}
// Fills `{name}` holes from `vars`; an unknown hole is left visible.
function tv(key, vars, lang) {
  return t(key, lang).replace(/\{(\w+)\}/g,
    (whole, name) => (vars && name in vars) ? String(vars[name]) : whole);
}
function applyLang(lang, root) {
  const d = root || document;
  if (d.documentElement) {
    d.documentElement.lang = lang === 'zh' ? 'zh' : 'en';
    d.documentElement.dataset.lang = lang;  // ends the pre-apply hiding (controls.css)
  }
  d.querySelectorAll('[data-i18n]').forEach(el => {
    let vars = null;
    try { vars = el.dataset.i18nVars ? JSON.parse(el.dataset.i18nVars) : null; } catch (_) {}
    el.textContent = vars ? tv(el.dataset.i18n, vars, lang) : t(el.dataset.i18n, lang);
  });
  // The switch names the other language, in that language.
  d.querySelectorAll('[data-i18n-other]').forEach(el => {
    el.textContent = t(el.dataset.i18nOther, lang === 'zh' ? 'en' : 'zh');
  });
  d.querySelectorAll('[data-i18n-title]').forEach(el => { el.title = t(el.dataset.i18nTitle, lang); });
  d.querySelectorAll('[data-i18n-ph]').forEach(el => { el.placeholder = t(el.dataset.i18nPh, lang); });
  d.querySelectorAll('[data-i18n-aria]').forEach(el => {
    el.setAttribute('aria-label', t(el.dataset.i18nAria, lang));
  });
}
function langSet(lang) {
  try { localStorage.setItem('sceneLang', lang); } catch (_) {}
  applyLang(lang);
  document.querySelectorAll('iframe').forEach(f => {
    try { f.contentWindow.postMessage({sceneLang: lang}, '*'); } catch (_) {}
  });
}
// A frame applies a language change sent by the shell.
window.addEventListener('message', e => {
  const lang = e.data && e.data.sceneLang;
  if (LANGS.includes(lang)) {
    try { localStorage.setItem('sceneLang', lang); } catch (_) {}
    applyLang(lang);
  }
});
// Apply once the markup exists; until then controls.css hides the English
// defaults, so a Chinese page never flashes English.
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', () => applyLang(langGet()));
} else {
  applyLang(langGet());
}
