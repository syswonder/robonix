// Class colours and point-feature label placement for the map canvases.

const CLS_COLORS = {
    robot: '#7aa7ff', table: '#f0c674', chair: '#e9b06b', monitor: '#88c0d0',
    person: '#f55', cup: '#a3be8c', bottle: '#a3be8c', tray: '#d08770',
    door: '#bf616a', plant: '#a3be8c', cabinet: '#d08770',
    keyboard: '#88c0d0', book: '#88c0d0', light_fixture: '#ebcb8b',
};

// Listed classes keep their colour; any other class gets a hue from FNV-1a
// over its name, so the same class looks the same on every machine and run.
function classColor(cls) {
  if (CLS_COLORS[cls]) return CLS_COLORS[cls];
  if (!cls) return '#9aa0a6';
  let h = 0x811c9dc5;
  for (let i = 0; i < cls.length; i++) {
    h ^= cls.charCodeAt(i);
    h = Math.imul(h, 0x01000193) >>> 0;
  }
  return `hsl(${((h % 3600) / 10).toFixed(1)}, 42%, 68%)`;
}

// Label placement by simulated annealing over eight positions around each
// dot (Christensen, Marks & Shieber, ACM TOG 14(3), 1995). Inline because
// the robot may have no network to fetch a library from.
const LBL = {
  // Up-right first: where a reader expects a label.
  CANDIDATES: [[1, -1], [1, 0], [1, 1], [0, -1], [0, 1], [-1, -1], [-1, 0], [-1, 1]],
  PAD: 3, SWEEPS: 36, T0: 1.0, COOL: 0.92,
};

function lblBox(a, pos) {
  const w = a.w + LBL.PAD * 2, h = a.h + LBL.PAD * 2, gap = a.r + 4;
  return {x: a.x + pos[0] * (gap + w / 2) - w / 2,
          y: a.y + pos[1] * (gap + h / 2) - h / 2, w, h};
}

function lblOverlap(p, q) {
  const dx = Math.min(p.x + p.w, q.x + q.w) - Math.max(p.x, q.x);
  const dy = Math.min(p.y + p.h, q.y + q.h) - Math.max(p.y, q.y);
  return (dx > 0 && dy > 0) ? dx * dy : 0;
}

// Overlap with other labels (per unit of own area), covering another dot,
// distance from the own dot (in label heights), and leaving the canvas.
function lblCost(boxes, anchors, i, W, H) {
  const b = boxes[i];
  let cost = 0;
  boxes.forEach((o, j) => { if (j !== i) cost += 2.4 * lblOverlap(b, o) / (b.w * b.h); });
  anchors.forEach((a, j) => {
    if (j !== i && a.x > b.x - a.r && a.x < b.x + b.w + a.r &&
        a.y > b.y - a.r && a.y < b.y + b.h + a.r) cost += 1.6;
  });
  const a = anchors[i];
  cost += 0.55 * Math.hypot(b.x + b.w / 2 - a.x, b.y + b.h / 2 - a.y) / b.h;
  if (b.x < 2 || b.y < 2 || b.x + b.w > W - 2 || b.y + b.h > H - 2) cost += 6;
  return cost;
}

// anchors: [{x, y, r, text, w, h}] in canvas pixels.
function placeLabels(anchors, W, H) {
  const boxes = anchors.map(a => lblBox(a, LBL.CANDIDATES[0]));
  let T = LBL.T0;
  for (let sweep = 0; sweep < LBL.SWEEPS; sweep++) {
    for (let i = 0; i < anchors.length; i++) {
      const before = lblCost(boxes, anchors, i, W, H), keep = boxes[i];
      boxes[i] = lblBox(anchors[i],
        LBL.CANDIDATES[(Math.random() * LBL.CANDIDATES.length) | 0]);
      const d = lblCost(boxes, anchors, i, W, H) - before;
      // Uphill moves pass with e^(-d/T): what gets it out of local minima.
      if (d > 0 && Math.random() >= Math.exp(-d / T)) boxes[i] = keep;
    }
    T *= LBL.COOL;
  }
  return boxes;
}

// Re-solved only when the objects or the view change. While the pointer is
// down a pan moves every anchor by one offset, so the last layout is shifted
// instead: re-solving is not continuous and would make labels jump.
// On `window` so the page can invalidate it.
window.lblCache = {key: '', boxes: null, shape: '', at: null};

function placedLabels(anchors, W, H, viewKey, frozen) {
  const cache = window.lblCache;
  const key = viewKey + '|' + anchors.map(a => a.text + ':' + (a.x | 0) + ',' + (a.y | 0)).join(';');
  if (key === cache.key) return cache.boxes;
  const shape = anchors.map(a => a.text).join(';');
  let boxes;
  if (frozen && cache.boxes && cache.at && shape === cache.shape
      && cache.at.length === anchors.length && anchors.length > 0) {
    const dx = anchors[0].x - cache.at[0].x, dy = anchors[0].y - cache.at[0].y;
    boxes = cache.boxes.map(b => Object.assign({}, b, {x: b.x + dx, y: b.y + dy}));
  } else {
    boxes = placeLabels(anchors, W, H);
  }
  window.lblCache = {key, boxes, shape, at: anchors.map(a => ({x: a.x, y: a.y}))};
  return boxes;
}

// A leader from the nearest box edge to the dot, only if the label moved away.
function drawLeader(ctx, box, a) {
  if (Math.hypot(box.x + box.w / 2 - a.x, box.y + box.h / 2 - a.y) < a.r + box.h) return;
  ctx.save();
  ctx.strokeStyle = 'rgba(255,255,255,0.38)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(Math.max(box.x, Math.min(a.x, box.x + box.w)),
             Math.max(box.y, Math.min(a.y, box.y + box.h)));
  ctx.lineTo(a.x, a.y);
  ctx.stroke();
  ctx.restore();
}
