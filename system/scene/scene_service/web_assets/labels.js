// Shared point-feature labelling: the class palette, and the placement of
// labels around the dots that carry it.
//
// Both maps draw the same objects over the same occupancy grid and both need
// a label that does not land on another label or on somebody else's dot.
// This lived in the 2D page while the regions page made do with anonymous
// dots -- which is the page where naming matters most, since marking a
// region means drawing a boundary around named things.

const CLS_COLORS = {
    robot: '#7aa7ff', table: '#f0c674', chair: '#e9b06b', monitor: '#88c0d0',
    person: '#f55', cup: '#a3be8c', bottle: '#a3be8c', tray: '#d08770',
    door: '#bf616a', plant: '#a3be8c', cabinet: '#d08770',
    keyboard: '#88c0d0', book: '#88c0d0', light_fixture: '#ebcb8b',
};

// FNV-1a over the class name. Written out rather than using anything
// built in, because the requirement is that the same class is the same
// colour on every machine and after every restart -- a per-process hash
// gives a palette that changes under you and makes two screenshots
// impossible to compare.
function clsHue(cls) {
  let h = 0x811c9dc5;
  for (let i = 0; i < cls.length; i++) {
    h ^= cls.charCodeAt(i);
    h = Math.imul(h, 0x01000193) >>> 0;
  }
  return (h % 3600) / 10;
}

function classColor(cls) {
  // The listed classes are deliberate -- `person` red and `door` red say
  // something a hash cannot -- so they win. Everything else gets a hue of
  // its own instead of the single grey they all used to share.
  if (CLS_COLORS[cls]) return CLS_COLORS[cls];
  if (!cls) return '#9aa0a6';
  // Held to one lightness and saturation so no class shouts over the rest;
  // only the hue carries the identity.
  return `hsl(${clsHue(cls).toFixed(1)}, 42%, 68%)`;
}

// ── Point-feature label placement ────────────────────────────────────────
// Simulated annealing over label offsets, after Christensen, Marks & Shieber,
// "An empirical study of algorithms for point-feature label placement"
// (ACM Transactions on Graphics 14(3), 1995), which is also what d3-labeler
// implements. Inline rather than vendored: this runs on a robot with no
// network, and a library that cannot be fetched when it is needed is not a
// dependency.
//
// Anchors are the object dots; each label starts at the up-right position
// the old code used and is free to move. Cost is overlap area with other
// labels, overlap with any dot, and how far the label sits from its own
// anchor. Annealing accepts a worsening move with probability e^(-dC/T),
// which is what lets it out of the local minimum a greedy pass settles into
// when three labels want the same gap.
const LBL = {
  // Eight positions around the anchor, in the order a reader prefers them:
  // up-right first, because that is where a label is expected, then around.
  CANDIDATES: [
    [ 1, -1], [ 1,  0], [ 1,  1], [ 0, -1],
    [ 0,  1], [-1, -1], [-1,  0], [-1,  1],
  ],
  PAD: 3,          // px of air around a label box before it counts as touching
  SWEEPS: 36,      // annealing sweeps; each visits every label once
  T0: 1.0,         // starting temperature, in units of the cost function
  COOL: 0.92,
};

function lblBox(a, pos, mw, mh) {
  // `pos` is [dx, dy] in units of the anchor's radius plus half the label.
  const gap = a.r + 4;
  const x = a.x + pos[0] * (gap + mw / 2) - mw / 2;
  const y = a.y + pos[1] * (gap + mh / 2) - mh / 2;
  return {x: x, y: y, w: mw, h: mh};
}

function lblOverlap(p, q) {
  const dx = Math.min(p.x + p.w, q.x + q.w) - Math.max(p.x, q.x);
  const dy = Math.min(p.y + p.h, q.y + q.h) - Math.max(p.y, q.y);
  return (dx > 0 && dy > 0) ? dx * dy : 0;
}

function lblCost(boxes, anchors, i, W, H) {
  const b = boxes[i];
  let cost = 0;
  for (let j = 0; j < boxes.length; j++) {
    if (j === i) continue;
    // Overlap is normalised by the label's own area so the term is
    // comparable across font sizes rather than tuned to one.
    cost += 2.4 * lblOverlap(b, boxes[j]) / (b.w * b.h);
  }
  for (let j = 0; j < anchors.length; j++) {
    const a = anchors[j];
    // A label sitting on another object's dot hides a thing it does not even
    // name, which is worse than sitting on another label.
    if (a.x > b.x - a.r && a.x < b.x + b.w + a.r &&
        a.y > b.y - a.r && a.y < b.y + b.h + a.r) {
      cost += (j === i) ? 0 : 1.6;
    }
  }
  // Distance from the anchor, in label-heights: a label far from its dot
  // needs a leader line, and a leader line is a cost to the reader.
  const a = anchors[i];
  const cx = b.x + b.w / 2, cy = b.y + b.h / 2;
  cost += 0.55 * Math.hypot(cx - a.x, cy - a.y) / b.h;
  // Off-canvas is not a placement.
  if (b.x < 2 || b.y < 2 || b.x + b.w > W - 2 || b.y + b.h > H - 2) cost += 6;
  return cost;
}

function placeLabels(anchors, W, H) {
  // anchors: [{x, y, r, text, w, h}] in canvas pixels.
  const n = anchors.length;
  const idx = new Array(n).fill(0);
  const boxes = anchors.map((a, i) =>
    lblBox(a, LBL.CANDIDATES[0], a.w + LBL.PAD * 2, a.h + LBL.PAD * 2));

  let T = LBL.T0;
  for (let sweep = 0; sweep < LBL.SWEEPS; sweep++) {
    for (let i = 0; i < n; i++) {
      const before = lblCost(boxes, anchors, i, W, H);
      const keepIdx = idx[i], keepBox = boxes[i];
      const cand = LBL.CANDIDATES[
        (Math.random() * LBL.CANDIDATES.length) | 0];
      idx[i] = LBL.CANDIDATES.indexOf(cand);
      boxes[i] = lblBox(anchors[i], cand,
                        anchors[i].w + LBL.PAD * 2, anchors[i].h + LBL.PAD * 2);
      const after = lblCost(boxes, anchors, i, W, H);
      const d = after - before;
      // Uphill moves are accepted with e^(-d/T): the escape hatch a greedy
      // pass does not have, and the reason this beats it when labels are
      // dense.
      if (d > 0 && Math.random() >= Math.exp(-d / T)) {
        idx[i] = keepIdx; boxes[i] = keepBox;
      }
    }
    T *= LBL.COOL;
  }
  return boxes;
}

// Placement is not cheap and the map redraws five times a second. It is also
// not needed that often: it only changes when the objects change or the view
// moves. A crawling label is harder to read than an overlapping one.
// Exposed deliberately: the placement is the one part of this page a
// screenshot cannot check, so the UI test reads the solved boxes.
let lblCache = {key: '', boxes: null, shape: '', at: null};
window.lblCache = lblCache;

// The identity of the label set, independent of where the view puts it. Two
// frames of the same pan share it; renaming or adding an object does not.
function labelShape(anchors) {
  return anchors.map(a => a.text).join(';');
}

function placedLabels(anchors, W, H, viewKey, frozen) {
  const key = viewKey + '|' + anchors.map(
    a => a.text + ':' + (a.x | 0) + ',' + (a.y | 0)).join(';');
  if (key === lblCache.key) return lblCache.boxes;

  const shape = labelShape(anchors);
  // While the pointer is down, translate rather than re-solve. Placement is
  // an annealing solve and is not continuous in its input: nudging every
  // anchor two pixels does not nudge the layout two pixels, it produces a
  // different one, and at five frames a second that reads as the labels
  // scattering. Under a pan every anchor moves by the same offset, so the
  // previous layout shifted by it is both correct and stable.
  if (frozen && lblCache.boxes && lblCache.at
      && shape === lblCache.shape
      && lblCache.at.length === anchors.length
      && anchors.length > 0) {
    const dx = anchors[0].x - lblCache.at[0].x;
    const dy = anchors[0].y - lblCache.at[0].y;
    const boxes = lblCache.boxes.map(
      b => Object.assign({}, b, {x: b.x + dx, y: b.y + dy}));
    lblCache = {key: key, boxes: boxes, shape: shape,
                at: anchors.map(a => ({x: a.x, y: a.y}))};
    window.lblCache = lblCache;
    return boxes;
  }

  lblCache = {
    key: key,
    boxes: placeLabels(anchors, W, H),
    shape: shape,
    at: anchors.map(a => ({x: a.x, y: a.y})),
  };
  window.lblCache = lblCache;
  return lblCache.boxes;
}

function drawLeader(ctx, box, a) {
  // Only when the label has actually moved away from its dot: a leader to a
  // label that is already touching it is a line that says nothing.
  const cx = box.x + box.w / 2, cy = box.y + box.h / 2;
  if (Math.hypot(cx - a.x, cy - a.y) < a.r + box.h) return;
  // From the box edge nearest the anchor, so the line does not cross the
  // text it belongs to.
  const ex = Math.max(box.x, Math.min(a.x, box.x + box.w));
  const ey = Math.max(box.y, Math.min(a.y, box.y + box.h));
  ctx.save();
  ctx.strokeStyle = 'rgba(255,255,255,0.38)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(ex, ey);
  ctx.lineTo(a.x, a.y);
  ctx.stroke();
  ctx.restore();
}
