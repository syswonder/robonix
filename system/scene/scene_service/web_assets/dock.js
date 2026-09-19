
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

    function renderDetail(force) {
      if (!force && detailBusy()) return;
      markSelected();
      if (!selectedId) { detail.hidden = true; detail.innerHTML = ''; return; }
      const o = lastObjects.find(x => x.id === selectedId);
      detail.hidden = false;
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
        <div class="acts">
          <button class="ren" data-i18n="dock.rename"></button>
          <button class="danger del" data-i18n="dock.delete"></button>
        </div>
        <div class="said" id="dock-said"></div>
      </div>`;
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
      box.innerHTML = `<input value="${o.cls}" />
        <button class="ok" data-i18n="dock.save"></button>
        <button class="no" data-i18n="dock.cancel"></button>`;
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
      acts.innerHTML = `<button class="confirm" data-i18n="dock.delete"></button>
        <button class="no" data-i18n="dock.cancel"></button>`;
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
          document.getElementById('dock-stamp').textContent =
            `${objs.length}${unsure ? ' · ' + unsure + '?' : ''}`;

          lastObjects = objs;
          const tb = document.getElementById('dock-objs');
          tb.innerHTML = objs.length ? objs.map(o => `
            <tr class="row ${Number(o.confidence) < UNSURE ? 'unsure' : ''}"
                data-oid="${o.id}">
              <td class="nm">${o.short_id}</td>
              <td class="cls">${o.cls}</td>
              <td class="pp ${o.missing ? 'miss' : ''}">
                ${fmt(o.pose.x)}, ${fmt(o.pose.y)}
              </td>
            </tr>`).join('')
            : `<tr><td class="empty">${t('dock.empty.objects')}</td></tr>`;
          tb.querySelectorAll('tr.row').forEach(tr => {
            tr.addEventListener('click', () => openDetail(tr.dataset.oid));
          });
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
          rel.innerHTML = edges.length ? edges.map(e => `
            <div class="rel">
              <span class="rs">${shortId(e.source_id)}</span>
              <span class="rp">${e.relation}</span>
              <span class="rt">${shortId(e.target_id)}</span>
            </div>`).join('')
            : `<span class="empty">${t('dock.empty.relations')}</span>`;

          const rb = document.getElementById('dock-robot');
          rb.innerHTML = s.robot ? `
            <div class="kv"><span class="k">x</span><span class="v">${fmt(s.robot.x)}</span></div>
            <div class="kv"><span class="k">y</span><span class="v">${fmt(s.robot.y)}</span></div>
            <div class="kv"><span class="k">z</span><span class="v">${fmt(s.robot.z)}</span></div>
            <div class="kv"><span class="k">yaw</span><span class="v">${fmt(s.robot.yaw)}</span></div>`
            : `<span class="empty">${t('dock.empty.robot')}</span>`;
        }
      } catch (_) { /* swallow; next tick will retry */ }
      setTimeout(tick, 500);
    }
    tick();
