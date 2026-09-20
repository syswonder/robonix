
    // ── The dock: tabs, collapse, resize, and the state poll ──
    const dock = document.getElementById('dock');
    const dockName = document.getElementById('dock-name');
    const LS = 'sceneDock.v2';
    const TAB_KEY = {objects: 'dock.objects', relations: 'dock.relations',
                     robot: 'dock.robot'};

    function save(patch) {
      try {
        const s = Object.assign(load(), patch);
        localStorage.setItem(LS, JSON.stringify(s));
      } catch (_) {}
    }
    function load() {
      try { return JSON.parse(localStorage.getItem(LS) || '{}'); }
      catch (_) { return {}; }
    }

    function show(tab) {
      dock.querySelectorAll('.tabs button').forEach(
        b => b.classList.toggle('on', b.dataset.tab === tab));
      dock.querySelectorAll('.pane').forEach(
        p => p.classList.toggle('on', p.dataset.pane === tab));
      dockName.dataset.i18n = TAB_KEY[tab] || '';
      dockName.textContent = t(TAB_KEY[tab] || tab);
      save({tab: tab});
    }

    dock.querySelectorAll('.tabs button').forEach(b => {
      b.addEventListener('click', () => {
        // Clicking a tab on the collapsed rail opens the dock on that tab:
        // the strip is both the switch and the way back, so collapsing is
        // never something you have to undo through a button elsewhere.
        const wasShut = dock.classList.contains('shut');
        const same = b.classList.contains('on');
        if (wasShut) { dock.classList.remove('shut'); save({shut: false}); }
        else if (same) { dock.classList.add('shut'); save({shut: true}); return; }
        show(b.dataset.tab);
      });
    });
    document.getElementById('dock-shut').addEventListener('click', () => {
      dock.classList.add('shut');
      save({shut: true});
    });

    // Resize by dragging the inner edge. Bounded so the dock can neither
    // vanish nor take the view it annotates.
    const grip = document.getElementById('dock-grip');
    let drag = null;
    grip.addEventListener('pointerdown', e => {
      drag = {x: e.clientX, w: dock.getBoundingClientRect().width};
      grip.setPointerCapture(e.pointerId);
      grip.classList.add('live');
      e.preventDefault();
    });
    grip.addEventListener('pointermove', e => {
      if (!drag) return;
      const w = Math.max(220, Math.min(drag.w + (drag.x - e.clientX),
                                       Math.round(window.innerWidth * 0.5)));
      dock.style.setProperty('--dock-w', w + 'px');
    });
    grip.addEventListener('pointerup', e => {
      if (!drag) return;
      drag = null;
      grip.classList.remove('live');
      try { grip.releasePointerCapture(e.pointerId); } catch (_) {}
      save({w: dock.style.getPropertyValue('--dock-w')});
    });

    (function restore() {
      const s = load();
      if (s.w) dock.style.setProperty('--dock-w', s.w);
      if (s.shut) dock.classList.add('shut');
      show(s.tab || 'objects');
    })();


    // ── Writing only what differs ──
    // Assigning the same string to a Text node still tears down the
    // selection sitting in it, so every write is guarded by a comparison.
    function setText(el, text) {
      if (el && el.textContent !== text) el.textContent = text;
    }
    function setClass(el, name, on) {
      if (el && el.classList.contains(name) !== !!on) {
        el.classList.toggle(name, !!on);
      }
    }

    function objectRow(o) {
      const tr = document.createElement('tr');
      tr.className = 'row';
      tr.dataset.oid = o.id;
      tr.innerHTML = '<td class="nm"></td><td class="cls"></td><td class="pp"></td>';
      tr.addEventListener('click', () => openDetail(tr.dataset.oid));
      return tr;
    }

    function fillRow(tr, o) {
      const unsure = Number(o.confidence) < UNSURE;
      setClass(tr, 'unsure', unsure);
      setText(tr.children[0], o.short_id);
      setText(tr.children[1], o.cls);
      setText(tr.children[2], `${fmt(o.pose.x)}, ${fmt(o.pose.y)}`);
      setClass(tr.children[2], 'miss', !!o.missing);
    }

    // Keyed by object id: what stayed is updated, what arrived is inserted,
    // what went is removed. Order follows the sorted list, so a row that
    // changes class moves rather than being rebuilt somewhere else.
    function syncObjects(tbody, objs) {
      const existing = new Map();
      tbody.querySelectorAll('tr.row').forEach(tr => existing.set(tr.dataset.oid, tr));
      const placeholder = tbody.querySelector('tr:not(.row)');
      if (objs.length && placeholder) placeholder.remove();
      if (!objs.length) {
        existing.forEach(tr => tr.remove());
        if (!placeholder) {
          const tr = document.createElement('tr');
          tr.innerHTML = `<td class="empty">${t('dock.empty.objects')}</td>`;
          tbody.appendChild(tr);
        } else {
          setText(placeholder.firstElementChild, t('dock.empty.objects'));
        }
        return;
      }
      let prev = null;
      for (const o of objs) {
        let tr = existing.get(o.id);
        if (tr) { existing.delete(o.id); } else { tr = objectRow(o); }
        fillRow(tr, o);
        const after = prev ? prev.nextSibling : tbody.firstChild;
        if (after !== tr) tbody.insertBefore(tr, after);
        prev = tr;
      }
      existing.forEach(tr => tr.remove());
    }

    // ── The detail view ──
    // One object, and the two corrections perception makes necessary often
    // enough that having to leave the panel to apply them is the wrong shape.
    let lastObjects = [];
    let lastBinding = null;
    let selectedId = null;
    const detail = document.getElementById('dock-detail');

    function openDetail(oid) {
      // Clicking the open row closes it, so the control is its own undo.
      selectedId = (selectedId === oid) ? null : oid;
      renderDetail();
    }
    function closeDetail() { selectedId = null; renderDetail(); }
    function markSelected() {
      document.querySelectorAll('#dock-objs tr.row').forEach(
        tr => tr.classList.toggle('sel', tr.dataset.oid === selectedId));
    }
    // The 2D map opens objects here too; it is the view we draw ourselves,
    // so a click on a dot can say which object it was.
    window.addEventListener('message', e => {
      const oid = e.data && e.data.sceneSelect;
      if (oid) { show('objects'); openDetail(oid); }
    });

    function say(text, el) { el.textContent = text || ''; }

    function detailBusy() {
      // An open rename field or delete confirmation means the reader owns
      // this pane until they finish or cancel.
      return !!(detail.querySelector('.edit')
                || detail.querySelector('.acts[data-asking]'));
    }

    // Which object the current DOM was built for. While it matches, the
    // poll only assigns values; the structure is left alone, so the fade
    // does not replay and a selection inside it survives.
    let renderedId = null;


    // Which object the evidence on screen belongs to, so a slow response
    // that arrives after the reader has moved on is dropped instead of
    // painting the previous object's pictures under the current one's name.
    let evidenceFor = null;

    async function loadViews(objectId) {
      evidenceFor = objectId;
      const strip = detail.querySelector('#dock-strip');
      if (!strip) return;
      let rows = [];
      try {
        const r = await fetch(
          `/api/objects/${encodeURIComponent(objectId)}/views`,
          {cache: 'no-store'});
        if (r.ok) rows = (await r.json()).views || [];
      } catch (_) { /* no pictures is a state, not an error */ }
      if (evidenceFor !== objectId) return;
      const hero = detail.querySelector('#dock-hero');
      if (!rows.length) {
        // Said plainly rather than left blank: an empty frame reads as a
        // panel that failed, and "not photographed yet" is information --
        // the robot has not been round the other side of it.
        if (hero) hero.innerHTML = `<div class="none" data-i18n="dock.noViews"></div>`;
        strip.innerHTML = '';
        applyLang(langGet(), document);
        return;
      }

      function show(row) {
        if (!hero) return;
        hero.innerHTML = `<img src="${row.url}" alt="">`;
      }

      // Only when there is a choice to make. One picture needs no selector,
      // and a row of one is furniture.
      strip.innerHTML = rows.length > 1 ? rows.map((v, i) =>
        `<button class="shot${i ? '' : ' on'}" data-url="${v.url}"
                 title="${t('dock.viewOf')} ${(v.bearing * 57.3).toFixed(0)}°">
           <img src="${v.url}" alt="">
         </button>`).join('') : '';
      strip.querySelectorAll('.shot').forEach((btn, i) => {
        btn.addEventListener('click', () => {
          strip.querySelectorAll('.shot').forEach(b => setClass(b, 'on', false));
          setClass(btn, 'on', true);
          show(rows[i]);
        });
      });
      show(rows[0]);
    }

    function updateDetail(o) {
      // Only the four values that move. Everything else -- the id, the
      // buttons, the labels -- was correct when it was built and rewriting
      // it would only cost the reader their selection.
      const unsure = Number(o.confidence) < UNSURE;
      const head = detail.querySelector('h3');
      const sub = detail.querySelector('.sub');
      if (head) setText(head, o.short_id);
      if (sub) setText(sub, o.cls + (unsure ? ' ?' : ''));
      const dds = detail.querySelectorAll('dl dd');
      // Order follows the markup below: id, confidence, observations, pose.
      if (dds[1]) {
        setText(dds[1], fmt(o.confidence));
        setClass(dds[1], 'warn', unsure);
      }
      if (dds[2]) setText(dds[2], String(o.observation_count ?? '—'));
      if (dds[3]) {
        setText(dds[3],
          `${fmt(o.pose.x)}, ${fmt(o.pose.y)}, ${fmt(o.pose.z ?? 0)}`);
      }
    }

    function renderDetail(force) {
      if (!force && detailBusy()) return;
      markSelected();
      if (!selectedId) {
        detail.hidden = true;
        detail.innerHTML = '';
        renderedId = null;
        return;
      }
      const o = lastObjects.find(x => x.id === selectedId);
      detail.hidden = false;
      if (o && renderedId === selectedId && detail.querySelector('.detail')) {
        updateDetail(o);
        return;
      }
      renderedId = o ? selectedId : null;
      if (!o) {
        // It was deleted, or the map changed under it. Say so rather than
        // leave a stale card that still offers to rename something gone.
        // Not an error: objects are re-registered as perception revises
        // itself, and the honest thing is to say the subject moved on.
        detail.innerHTML = `<div class="detail">
          <div class="said">${t('dock.gone')}</div></div>`;
        applyLang(langGet(), document);
        return;
      }
      const unsure = Number(o.confidence) < UNSURE;
      detail.innerHTML = `<div class="detail">
        <h3>${o.short_id}</h3>
        <div class="sub">${o.cls}${unsure ? ' ?' : ''}</div>
        <dl>
          <dt data-i18n="dock.id"></dt><dd>${o.id}</dd>
          <dt data-i18n="dock.conf"></dt>
          <dd class="${unsure ? 'warn' : ''}">${fmt(o.confidence)}</dd>
          <dt data-i18n="dock.obs"></dt><dd>${o.observation_count ?? '—'}</dd>
          <dt data-i18n="dock.pos"></dt>
          <dd>${fmt(o.pose.x)}, ${fmt(o.pose.y)}, ${fmt(o.pose.z ?? 0)}</dd>
          ${o.missing ? `<dt></dt><dd class="warn">${t('dock.missing')}</dd>` : ''}
        </dl>
        <div class="evidence">
          <div class="hero" id="dock-hero"></div>
          <div class="strip" id="dock-strip"></div>
        </div>
        <div class="acts">
          <button class="btn ren" data-i18n="dock.rename"></button>
          <button class="btn danger del" data-i18n="dock.delete"></button>
        </div>
        <div class="said" id="dock-said"></div>
      </div>`;
      // Once per selection, not per tick: the panel polls, and this is a
      // network read.
      loadViews(o.id);
      const said = detail.querySelector('#dock-said');
      detail.querySelector('.ren').addEventListener('click',
        () => rename(o, said));
      detail.querySelector('.del').addEventListener('click',
        () => remove(o, said));
      applyLang(langGet(), document);
    }

    // The epoch the page rendered travels with the edit, so a correction
    // aimed at this object cannot land on a different map after a switch.
    function epoch() {
      return {
        expected_map_id: (lastBinding && lastBinding.map_id) || '',
        expected_generation: (lastBinding && lastBinding.generation) ?? null,
      };
    }

    function rename(o, said) {
      // In place: the browser's prompt() is a different typeface, palette and
      // button order, and it suspends the page while it is open.
      const head = detail.querySelector('h3');
      if (!head || detail.querySelector('.edit')) return;
      const box = document.createElement('div');
      box.className = 'edit';
      box.innerHTML = `<input class="field" value="${o.cls}" />
        <button class="btn ok" data-i18n="dock.save"></button>
        <button class="btn no" data-i18n="dock.cancel"></button>`;
      head.replaceWith(box);
      const field = box.querySelector('input');
      const restore = () => { box.replaceWith(head); };
      const commit = () => {
        const label = field.value.trim();
        if (!label) { restore(); return; }
        restore();
        saveLabel(o, label, said);
      };
      box.querySelector('.ok').addEventListener('click', commit);
      box.querySelector('.no').addEventListener('click', restore);
      field.addEventListener('keydown', e => {
        if (e.key === 'Enter') { e.preventDefault(); commit(); }
        if (e.key === 'Escape') { e.preventDefault(); restore(); }
      });
      applyLang(langGet(), document);
      field.focus();
      field.select();
    }

    async function saveLabel(o, label, said) {
      try {
        const r = await fetch(`/api/objects/${encodeURIComponent(o.id)}/label`, {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(Object.assign({label: label}, epoch())),
        });
        const d = await r.json();
        say(d.ok ? '' : (d.detail || 'failed'), said);
      } catch (e) { say(String(e), said); }
    }

    function remove(o, said) {
      // Asks in the panel rather than through the browser, and the confirming
      // button is the red one.
      const acts = detail.querySelector('.acts');
      if (!acts || acts.dataset.asking) return;
      acts.dataset.asking = '1';
      const original = acts.innerHTML;
      say(t('dock.deleteAsk'), said);
      acts.innerHTML = `<button class="btn confirm" data-i18n="dock.delete"></button>
        <button class="btn no" data-i18n="dock.cancel"></button>`;
      const undo = () => {
        acts.innerHTML = original;
        delete acts.dataset.asking;
        say('', said);
        acts.querySelector('.ren').addEventListener('click',
          () => rename(o, said));
        acts.querySelector('.del').addEventListener('click',
          () => remove(o, said));
        applyLang(langGet(), document);
      };
      acts.querySelector('.no').addEventListener('click', undo);
      acts.querySelector('.confirm').addEventListener('click',
        () => doDelete(o, said));
      applyLang(langGet(), document);
    }

    async function doDelete(o, said) {
      try {
        const r = await fetch(`/api/objects/${encodeURIComponent(o.id)}`, {
          method: 'DELETE',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(epoch()),
        });
        const d = await r.json();
        if (d.ok) { closeDetail(); } else { say(d.detail || 'failed', said); }
      } catch (e) { say(String(e), said); }
    }

    // ── /api/state → the panes ──
    const fmt = n => Number(n).toFixed(2);
    const shortId = id => String(id).split('.').pop();
    // Below this, an object is a lead rather than a fact. Perception in a
    // room like this is not accurate enough to present every hit flatly, and
    // the honest UI marks the doubtful ones so the next step is to go and
    // look again.
    const UNSURE = 0.55;

    async function tick() {
      try {
        const r = await fetch('/api/state', {cache: 'no-store'});
        if (r.ok) {
          const s = await r.json();
          lastBinding = s.map_binding || null;
          const objs = (s.objects || []).slice().sort(
            (a, b) => a.cls.localeCompare(b.cls));
          const edges = (s.scene_graph && s.scene_graph.edges) || [];
          const unsure = objs.filter(o => Number(o.confidence) < UNSURE).length;

          // No timestamp: a unix float tells a reader nothing they can
          // act on and rewrites itself twice a second, which made the one
          // line meant to be stable chrome the busiest thing in the panel.
          setText(document.getElementById('dock-stamp'),
            `${objs.length}${unsure ? ' · ' + unsure + '?' : ''}`);

          lastObjects = objs;
          syncObjects(document.getElementById('dock-objs'), objs);
          if (selectedId) {
            // A selection that no longer exists outranks an open edit: the
            // thing being edited is gone, and the card would otherwise offer
            // to rename and delete a ghost.
            const alive = lastObjects.some(x => x.id === selectedId);
            if (!alive) {
              detail.querySelectorAll('.edit').forEach(e => e.remove());
              const acts = detail.querySelector('.acts[data-asking]');
              if (acts) delete acts.dataset.asking;
              renderDetail(true);
            } else if (!detailBusy()) {
              renderDetail();
            }
          }

          const rel = document.getElementById('dock-rels');
          const relHtml = edges.length ? edges.map(e => `
            <div class="rel">
              <span class="rs">${shortId(e.source_id)}</span>
              <span class="rp">${e.relation}</span>
              <span class="rt">${shortId(e.target_id)}</span>
            </div>`).join('')
            : `<span class="empty">${t('dock.empty.relations')}</span>`;
          // Written only when it differs: an identical assignment still
          // collapses any selection inside it.
          if (rel.innerHTML !== relHtml) rel.innerHTML = relHtml;

          const rb = document.getElementById('dock-robot');
          const robHtml = s.robot ? `
            <div class="kv"><span class="k">x</span><span class="v">${fmt(s.robot.x)}</span></div>
            <div class="kv"><span class="k">y</span><span class="v">${fmt(s.robot.y)}</span></div>
            <div class="kv"><span class="k">z</span><span class="v">${fmt(s.robot.z)}</span></div>
            <div class="kv"><span class="k">yaw</span><span class="v">${fmt(s.robot.yaw)}</span></div>`
            : `<span class="empty">${t('dock.empty.robot')}</span>`;
          if (rb.innerHTML !== robHtml) rb.innerHTML = robHtml;
        }
      } catch (_) { /* swallow; next tick will retry */ }
      setTimeout(tick, 500);
    }
    tick();
