
    // ── The dock: tabs, collapse, resize, and the state poll ──
    const dock = document.getElementById('dock');
    const dockName = document.getElementById('dock-name');
    const detail = document.getElementById('dock-detail');
    const LS = 'sceneDock.v2';
    const TAB_KEY = {objects: 'dock.objects', relations: 'dock.relations',
                     robot: 'dock.robot'};
    // Below this confidence an object is marked as a lead, not a fact.
    const UNSURE = 0.55;

    let lastObjects = [];
    let lastBinding = null;
    let selectedId = null;
    let renderedId = null;   // which object the detail DOM was built for
    let showUnsettled = false;

    const fmt = n => Number(n).toFixed(2);
    const esc = v => String(v ?? '').replace(/[&<>"']/g,
      ch => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'}[ch]));
    const unsure = o => Number(o.confidence) < UNSURE;
    const pose = o => `${fmt(o.pose.x)}, ${fmt(o.pose.y)}, ${fmt(o.pose.z ?? 0)}`;
    const captionBy = o => t(!o.caption ? 'dock.captionNone'
      : o.caption_source === 'operator' ? 'dock.captionYou' : 'dock.captionModel');
    const nameOf = id => {
      const o = lastObjects.find(x => x.id === id);
      return o ? o.display_name : String(id).split('.').pop();
    };
    const say = (text, el) => { el.textContent = text || ''; };
    const relocalize = () => applyLang(langGet(), document);

    function load() {
      try { return JSON.parse(localStorage.getItem(LS) || '{}'); }
      catch (_) { return {}; }
    }
    function save(patch) {
      try { localStorage.setItem(LS, JSON.stringify(Object.assign(load(), patch))); }
      catch (_) {}
    }

    // Writing an identical value still resets any text selection in it.
    function setText(el, text) {
      if (el && el.textContent !== text) el.textContent = text;
    }
    function setClass(el, name, on) {
      if (el) el.classList.toggle(name, !!on);
    }
    function setHtml(el, html) {
      if (el && el.innerHTML !== html) el.innerHTML = html;
    }

    function show(tab) {
      dock.querySelectorAll('.tabs button').forEach(b => setClass(b, 'on', b.dataset.tab === tab));
      dock.querySelectorAll('.pane').forEach(p => setClass(p, 'on', p.dataset.pane === tab));
      dock.dataset.tab = tab;
      dockName.dataset.i18n = TAB_KEY[tab] || '';
      dockName.textContent = t(TAB_KEY[tab] || tab);
      save({tab});
    }
    function shut() { dock.classList.add('shut'); save({shut: true}); }

    // On the collapsed rail a tab reopens the dock; the open tab collapses it.
    dock.querySelectorAll('.tabs button').forEach(b => b.addEventListener('click', () => {
      if (dock.classList.contains('shut')) { dock.classList.remove('shut'); save({shut: false}); }
      else if (b.classList.contains('on')) { shut(); return; }
      show(b.dataset.tab);
    }));
    document.getElementById('dock-shut').addEventListener('click', shut);

    // Drag with pointer capture; `size(start, e)` returns the new CSS length.
    // Both sizes are clamped so neither the dock nor the view can vanish.
    function draggable(handle, cssVar, storeKey, size) {
      if (!handle) return;
      let start = null;
      handle.addEventListener('pointerdown', e => {
        start = {x: e.clientX, w: dock.getBoundingClientRect().width,
                 box: handle.parentElement.getBoundingClientRect()};
        handle.setPointerCapture(e.pointerId);
        handle.classList.add('live');
        e.preventDefault();
      });
      handle.addEventListener('pointermove', e => {
        if (start) dock.style.setProperty(cssVar, Math.round(size(start, e)) + 'px');
      });
      const done = () => {
        if (!start) return;
        start = null;
        handle.classList.remove('live');
        save({[storeKey]: dock.style.getPropertyValue(cssVar)});
      };
      handle.addEventListener('pointerup', done);
      handle.addEventListener('pointercancel', done);
    }
    draggable(document.getElementById('dock-grip'), '--dock-w', 'w', (s, e) =>
      Math.max(220, Math.min(s.w + s.x - e.clientX, window.innerWidth * 0.5)));
    draggable(document.getElementById('dock-split'), '--detail-h', 'dh', (s, e) =>
      Math.max(120, Math.min(s.box.bottom - e.clientY, s.box.height - 80)));

    (function restore() {
      const s = load();
      if (s.w) dock.style.setProperty('--dock-w', s.w);
      if (s.dh) dock.style.setProperty('--detail-h', s.dh);
      if (s.shut) dock.classList.add('shut');
      show(s.tab || 'objects');
    })();

    // ── The object list ──
    // The list is the semantic map: settled objects. The rest sit behind a
    // toggle; the selected object stays listed whatever its state.
    const shownObjects = () => showUnsettled ? lastObjects
      : lastObjects.filter(o => o.settled || o.id === selectedId);

    function renderList() {
      syncObjects(document.getElementById('dock-objs'), shownObjects());
      const hidden = lastObjects.filter(o => !o.settled && o.id !== selectedId).length;
      const host = document.getElementById('dock-gone');
      if (!host) return;
      host.hidden = !hidden && !showUnsettled;
      setText(host, showUnsettled ? t('dock.goneHide') : tv('dock.goneShow', {n: hidden}));
    }
    document.getElementById('dock-gone')?.addEventListener('click', () => {
      showUnsettled = !showUnsettled;
      renderList();
    });

    function fillRow(tr, o) {
      setClass(tr, 'unsure', unsure(o));
      setClass(tr, 'gone', !o.settled);
      setText(tr.children[0], o.display_name);
      setText(tr.children[1], o.caption || o.label);
      setText(tr.children[2], o.region || '');
    }

    // Keyed by id, so rows are updated and moved rather than rebuilt.
    function syncObjects(tbody, objs) {
      const existing = new Map();
      tbody.querySelectorAll('tr.row').forEach(tr => existing.set(tr.dataset.oid, tr));
      let placeholder = tbody.querySelector('tr:not(.row)');
      if (!objs.length) {
        existing.forEach(tr => tr.remove());
        if (!placeholder) {
          placeholder = tbody.appendChild(document.createElement('tr'));
          placeholder.innerHTML = '<td class="empty"></td>';
        }
        setText(placeholder.firstElementChild, t('dock.empty.objects'));
        return;
      }
      if (placeholder) placeholder.remove();
      let prev = null;
      for (const o of objs) {
        let tr = existing.get(o.id);
        existing.delete(o.id);
        if (!tr) {
          tr = document.createElement('tr');
          tr.className = 'row';
          tr.dataset.oid = o.id;
          tr.innerHTML = '<td class="nm"></td><td class="cls"></td><td class="rg"></td>';
          tr.addEventListener('click', () => openDetail(o.id));
        }
        fillRow(tr, o);
        const after = prev ? prev.nextSibling : tbody.firstChild;
        if (after !== tr) tbody.insertBefore(tr, after);
        prev = tr;
      }
      existing.forEach(tr => tr.remove());
    }

    // ── The link to the rerun viewer ──
    // The viewer reports `selection_change` (relayed by the host page) but
    // has no setter, so the other direction asks scene to re-aim the camera.
    function objectIdFromPath(path) {
      // `/map/objects/<layer>/<id>` (3D) or `/map2d/objects/<layer>/<id>` (2D).
      const parts = String(path || '').split('/').filter(Boolean);
      if (parts.length < 2 || !['map', 'map2d'].includes(parts[0]) || parts[1] !== 'objects') {
        return null;
      }
      const last = parts[parts.length - 1];
      return last.startsWith('scene.object.') ? last : null;
    }
    function focusViewer(oid) {
      document.querySelectorAll('iframe').forEach(f => {
        try {
          f.contentWindow.postMessage({source: 'scene-shell', type: 'focus', object_id: oid},
                                      location.origin);
        } catch (_) { /* not our frame */ }
      });
    }
    window.addEventListener('message', ev => {
      const m = ev.data;
      if (ev.origin !== location.origin || !m || m.source !== 'scene-rerun'
          || m.type !== 'selection') return;
      const oid = objectIdFromPath(m.entity_path);
      if (!oid) return;
      // Set, not toggle: the viewer re-fires for every click on one entity.
      if (selectedId !== oid) { selectedId = oid; renderDetail(); }
      document.querySelector(`#dock-objs tr.row[data-oid="${CSS.escape(oid)}"]`)
        ?.scrollIntoView({block: 'nearest'});
    });

    // ── The detail pane ──
    function openDetail(oid) {
      selectedId = selectedId === oid ? null : oid;
      renderDetail();
      if (selectedId) focusViewer(selectedId);
    }
    function closeDetail() { selectedId = null; renderDetail(); }

    // An open edit field or delete confirmation owns the pane until finished.
    const detailBusy = () => !!detail.querySelector('.edit, .acts[data-asking]');

    // [string key, value, is it a warning]
    const FIELDS = [
      ['dock.region', o => o.region || '—'],
      ['dock.cls', o => o.label],
      ['dock.id', o => o.id],
      ['dock.conf', o => fmt(o.confidence), unsure],
      ['dock.obs', o => o.observation_count ?? '—'],
      ['dock.pos', pose],
    ];
    const fieldsHtml = o => FIELDS.map(([key, value, warn]) =>
      `<dt data-i18n="${key}">${t(key)}</dt><dd${warn && warn(o) ? ' class="warn"' : ''}>${esc(value(o))}</dd>`
    ).join('');

    // Only the values that move are rewritten, so a selection in the card
    // survives the poll.
    function updateDetail(o) {
      setText(detail.querySelector('h3'), o.display_name);
      const sub = detail.querySelector('.sub');
      setText(sub, (o.caption || o.label) + (unsure(o) ? ' ?' : ''));
      if (sub) sub.title = captionBy(o);
      setHtml(detail.querySelector('dl'), fieldsHtml(o));
    }

    function renderDetail(force) {
      if (!force && detailBusy()) return;
      document.querySelectorAll('#dock-objs tr.row').forEach(
        tr => setClass(tr, 'sel', tr.dataset.oid === selectedId));
      detail.hidden = !selectedId;
      if (!selectedId) { detail.innerHTML = ''; renderedId = null; return; }
      const o = lastObjects.find(x => x.id === selectedId);
      if (o && renderedId === selectedId && detail.querySelector('.detail')) {
        updateDetail(o);
        return;
      }
      renderedId = o ? selectedId : null;
      if (!o) {
        // Deleted, or re-registered after the map changed.
        detail.innerHTML = `<div class="detail"><div class="said">${t('dock.gone')}</div></div>`;
        relocalize();
        return;
      }
      detail.innerHTML = `<div class="detail">
        <h3>${esc(o.display_name)}</h3>
        <div class="sub" title="${esc(captionBy(o))}">${esc(o.caption || o.label)}${unsure(o) ? ' ?' : ''}</div>
        <div class="acts"></div>
        <div class="said" id="dock-said"></div>
        <dl>${fieldsHtml(o)}</dl>
        <div class="evidence">
          <div class="hero" id="dock-hero"></div>
          <div class="strip" id="dock-strip"></div>
        </div>
      </div>`;
      wireActs(o);
      loadViews(o.id);
    }

    function wireActs(o) {
      const acts = detail.querySelector('.acts');
      const said = detail.querySelector('#dock-said');
      acts.innerHTML = `<button class="btn cap" data-i18n="dock.describe"></button>
        <button class="btn ren" data-i18n="dock.rename"></button>
        <button class="btn danger del" data-i18n="dock.delete"></button>`;
      // The object as the last poll saw it, not as it was when this opened.
      const now = () => lastObjects.find(x => x.id === o.id) || o;
      acts.querySelector('.cap').addEventListener('click', () => describe(now(), said));
      acts.querySelector('.ren').addEventListener('click', () => rename(now(), said));
      acts.querySelector('.del').addEventListener('click', () => remove(o, said, acts));
      relocalize();
    }

    // Photographs of the object; a late response for another object is dropped.
    let evidenceFor = null;
    async function loadViews(objectId) {
      evidenceFor = objectId;
      let rows = [];
      try {
        const r = await fetch(`/api/objects/${encodeURIComponent(objectId)}/views`,
                              {cache: 'no-store'});
        if (r.ok) rows = (await r.json()).views || [];
      } catch (_) { /* no pictures is a state, not an error */ }
      const hero = detail.querySelector('#dock-hero');
      const strip = detail.querySelector('#dock-strip');
      if (evidenceFor !== objectId || !strip) return;
      const showView = row => { if (hero) hero.innerHTML = `<img src="${esc(row.url)}" alt="">`; };
      if (!rows.length) {
        if (hero) hero.innerHTML = '<div class="none" data-i18n="dock.noViews"></div>';
        strip.innerHTML = '';
        relocalize();
        return;
      }
      // A selector only when there is a choice.
      strip.innerHTML = rows.length < 2 ? '' : rows.map((v, i) =>
        `<button class="shot${i ? '' : ' on'}" title="${t('dock.viewOf')} ${(v.bearing * 57.3).toFixed(0)}°">
           <img src="${esc(v.url)}" alt=""></button>`).join('');
      strip.querySelectorAll('.shot').forEach((btn, i) => btn.addEventListener('click', () => {
        strip.querySelectorAll('.shot').forEach(b => setClass(b, 'on', b === btn));
        showView(rows[i]);
      }));
      showView(rows[0]);
    }

    // ── Corrections ──
    // The epoch the page rendered travels with an edit, so it cannot land on
    // a different map after a switch.
    const epoch = () => ({
      expected_map_id: (lastBinding && lastBinding.map_id) || '',
      expected_generation: (lastBinding && lastBinding.generation) ?? null,
    });

    async function send(url, method, fields, said, onOk) {
      try {
        const r = await fetch(url, {method, headers: {'Content-Type': 'application/json'},
                                    body: JSON.stringify(Object.assign(fields, epoch()))});
        const d = await r.json();
        if (d.ok) { say('', said); if (onOk) onOk(); }
        else say(d.detail || 'failed', said);
      } catch (e) { say(String(e), said); }
    }
    const objectUrl = o => `/api/objects/${encodeURIComponent(o.id)}`;

    // An in-page field rather than prompt(): `el` is swapped for it until the
    // edit is saved or dropped.
    function editInPlace(el, value, allowEmpty, onCommit) {
      if (!el || detail.querySelector('.edit')) return;
      const box = document.createElement('div');
      box.className = 'edit';
      box.innerHTML = `<input class="field" value="${esc(value)}" />
        <button class="btn ok" data-i18n="dock.save"></button>
        <button class="btn no" data-i18n="dock.cancel"></button>`;
      el.replaceWith(box);
      const field = box.querySelector('input');
      const restore = () => box.replaceWith(el);
      const commit = () => {
        const text = field.value.trim();
        restore();
        if (text || allowEmpty) onCommit(text);
      };
      box.querySelector('.ok').addEventListener('click', commit);
      box.querySelector('.no').addEventListener('click', restore);
      field.addEventListener('keydown', e => {
        if (e.isComposing) return;  // Enter picks an IME candidate
        if (e.key === 'Enter') { e.preventDefault(); commit(); }
        if (e.key === 'Escape') { e.preventDefault(); restore(); }
      });
      relocalize();
      field.focus();
      field.select();
    }

    function rename(o, said) {
      editInPlace(detail.querySelector('h3'), o.label, false,
        label => send(`${objectUrl(o)}/label`, 'POST', {label}, said));
    }
    // An empty caption hands the object back to the VLM.
    function describe(o, said) {
      editInPlace(detail.querySelector('.sub'), o.caption || '', true,
        caption => send(`${objectUrl(o)}/caption`, 'POST', {caption}, said));
    }
    // Confirmed in the panel, with the red button as the confirmation.
    function remove(o, said, acts) {
      if (acts.dataset.asking) return;
      acts.dataset.asking = '1';
      say(t('dock.deleteAsk'), said);
      acts.innerHTML = `<button class="btn confirm" data-i18n="dock.delete"></button>
        <button class="btn no" data-i18n="dock.cancel"></button>`;
      acts.querySelector('.no').addEventListener('click', () => {
        delete acts.dataset.asking;
        say('', said);
        wireActs(o);
      });
      acts.querySelector('.confirm').addEventListener('click',
        () => send(objectUrl(o), 'DELETE', {}, said, closeDetail));
      relocalize();
    }

    // ── Flush: drop every perceived object; confirmed by a second click ──
    const flushBtn = document.getElementById('dock-flush');
    if (flushBtn) {
      let armed = null;
      const reset = () => {
        clearTimeout(armed);
        armed = null;
        flushBtn.classList.remove('confirm');
        flushBtn.textContent = t('dock.flush');
      };
      flushBtn.addEventListener('click', async () => {
        const count = lastObjects.length;
        if (!armed) {
          flushBtn.classList.add('confirm');
          flushBtn.textContent = tv('dock.flushAsk', {n: count});
          armed = setTimeout(reset, 4000);
          return;
        }
        reset();
        flushBtn.disabled = true;
        try {
          const r = await fetch('/api/objects/flush', {
            method: 'POST', headers: {'Content-Type': 'application/json'},
            body: JSON.stringify(epoch())});
          const out = await r.json().catch(() => ({}));
          flushBtn.textContent = r.ok ? tv('dock.flushDone', {n: out.deleted ?? count})
                                      : (out.detail || String(r.status));
        } catch (err) {
          flushBtn.textContent = String(err);
        }
        setTimeout(() => { flushBtn.disabled = false; flushBtn.textContent = t('dock.flush'); }, 2500);
      });
    }

    // ── /api/state → the panes ──
    const kv = (k, v) => `<div class="kv"><span class="k">${k}</span><span class="v">${fmt(v)}</span></div>`;

    async function tick() {
      try {
        const r = await fetch('/api/state', {cache: 'no-store'});
        if (r.ok) {
          const s = await r.json();
          lastBinding = s.map_binding || null;
          lastObjects = (s.objects || []).slice().sort(
            (a, b) => a.display_name.localeCompare(b.display_name, undefined, {numeric: true}));
          const doubtful = lastObjects.filter(unsure).length;
          setText(document.getElementById('dock-stamp'),
            `${lastObjects.length}${doubtful ? ' · ' + doubtful + '?' : ''}`);
          renderList();
          if (selectedId) {
            if (!lastObjects.some(x => x.id === selectedId)) {
              // Merged into another record: follow it rather than say "gone".
              const was = selectedId;
              const moved = await fetch(`/api/objects/${encodeURIComponent(was)}/resolve`,
                                        {cache: 'no-store'}).then(r => r.json()).catch(() => ({}));
              if (moved.id && selectedId === was && lastObjects.some(x => x.id === moved.id)) {
                selectedId = moved.id;
              } else {
                // Gone: that outranks an open edit on it.
                detail.querySelectorAll('.edit').forEach(e => e.remove());
                const acts = detail.querySelector('.acts[data-asking]');
                if (acts) delete acts.dataset.asking;
              }
              renderDetail(true);
            } else {
              renderDetail();
            }
          }
          const edges = (s.scene_graph && s.scene_graph.edges) || [];
          setHtml(document.getElementById('dock-rels'), edges.length ? edges.map(e => `
            <div class="rel">
              <span class="rs">${esc(nameOf(e.source_id))}</span>
              <span class="rp">${esc(e.relation)}</span>
              <span class="rt">${esc(nameOf(e.target_id))}</span>
            </div>`).join('') : `<span class="empty">${t('dock.empty.relations')}</span>`);
          setHtml(document.getElementById('dock-robot'), s.robot
            ? ['x', 'y', 'z', 'yaw'].map(k => kv(k, s.robot[k])).join('')
            : `<span class="empty">${t('dock.empty.robot')}</span>`);
        }
      } catch (_) { /* the next tick retries */ }
      setTimeout(tick, 500);
    }
    tick();
