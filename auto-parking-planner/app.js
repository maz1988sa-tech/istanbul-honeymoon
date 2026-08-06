/* ═══════════════════════════════════════════════════════════════════════════
   Auto Parking Planner — app.js
   A self-contained, dependency-free parking-layout planning tool.

   Module map (all in this file, separated by banner comments):
     1. Util      — small helpers (debounce, formatting, ids, DOM)
     2. G         — pure 2-D geometry (points, polygons, SAT, clipping)
     3. State     — application state, defaults, undo/redo, presets
     4. Demand    — parking-demand calculation (methods A–D)
     5. Generator — geometric parking-layout generation + optimization
     6. Rules     — compliance / warnings engine
     7. Renderer  — SVG scene construction and drawing
     8. Interact  — pointer tools: pan/zoom, drag, draw, measure, edit
     9. UI        — data binding, dynamic lists, KPIs, presets, exports
    10. App       — bootstrapping and the recalculation pipeline

   Coordinate system: world units are METRES. x grows east (right),
   y grows south (down). North is −y unless the user rotates north.
   ═══════════════════════════════════════════════════════════════════════════ */
'use strict';

/* ═══════════════════════ 1. Util ═══════════════════════ */

const Util = {
  /** Debounce fn by ms. */
  debounce(fn, ms) {
    let t = null;
    return function (...args) {
      clearTimeout(t);
      t = setTimeout(() => fn.apply(this, args), ms);
    };
  },
  clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); },
  /** Format a number with fixed decimals, trimming trailing decimal zeros. */
  fmt(v, dec = 2) {
    if (!isFinite(v)) return '–';
    const s = v.toFixed(dec);
    return s.indexOf('.') === -1 ? s : s.replace(/0+$/, '').replace(/\.$/, '');
  },
  fmtArea(v) { return isFinite(v) ? Util.fmt(v, 1) + ' m²' : '–'; },
  fmtLen(v) { return isFinite(v) ? Util.fmt(v, 2) + ' m' : '–'; },
  uid(prefix) { Util._uidCounter = (Util._uidCounter || 0) + 1; return prefix + '_' + Util._uidCounter + '_' + Math.random().toString(36).slice(2, 7); },
  deepClone(o) { return JSON.parse(JSON.stringify(o)); },
  el(id) { return document.getElementById(id); },
  /** Create an SVG element with attributes. */
  svgEl(tag, attrs, parent) {
    const e = document.createElementNS('http://www.w3.org/2000/svg', tag);
    if (attrs) for (const k in attrs) e.setAttribute(k, attrs[k]);
    if (parent) parent.appendChild(e);
    return e;
  },
  /** Set many attributes at once. */
  attr(e, attrs) { for (const k in attrs) e.setAttribute(k, attrs[k]); return e; },
  /** Download a text/blob as a file. */
  download(filename, data, mime) {
    const blob = data instanceof Blob ? data : new Blob([data], { type: mime || 'text/plain' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url; a.download = filename;
    document.body.appendChild(a);
    a.click();
    setTimeout(() => { document.body.removeChild(a); URL.revokeObjectURL(url); }, 100);
  },
  todayISO() { const d = new Date(); return d.toISOString().slice(0, 10); },
  escapeHTML(s) {
    return String(s == null ? '' : s).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
  }
};

/* ═══════════════════════ 2. G — geometry ═══════════════════════ */

const G = {
  EPS: 1e-9,
  /** Overlap tolerance in metres: penetrations smaller than this are treated
      as touching, not intersecting (stalls may share edges with aisles). */
  TOL: 0.015,

  d2r(d) { return d * Math.PI / 180; },
  r2d(r) { return r * 180 / Math.PI; },

  add(a, b) { return { x: a.x + b.x, y: a.y + b.y }; },
  sub(a, b) { return { x: a.x - b.x, y: a.y - b.y }; },
  scale(a, k) { return { x: a.x * k, y: a.y * k }; },
  dot(a, b) { return a.x * b.x + a.y * b.y; },
  cross(a, b) { return a.x * b.y - a.y * b.x; },
  len(a) { return Math.hypot(a.x, a.y); },
  dist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); },
  norm(a) { const l = Math.hypot(a.x, a.y) || 1; return { x: a.x / l, y: a.y / l }; },
  perp(a) { return { x: -a.y, y: a.x }; },

  /** Rotate point p by ang (radians) around origin. */
  rot(p, ang) {
    const c = Math.cos(ang), s = Math.sin(ang);
    return { x: p.x * c - p.y * s, y: p.x * s + p.y * c };
  },
  /** Rotate point p by ang (radians) around centre c. */
  rotAround(p, c, ang) {
    const r = G.rot({ x: p.x - c.x, y: p.y - c.y }, ang);
    return { x: r.x + c.x, y: r.y + c.y };
  },

  /** Signed polygon area (shoelace). Positive when vertices wind clockwise
      in this y-down coordinate system. */
  polygonSignedArea(pts) {
    let s = 0;
    for (let i = 0; i < pts.length; i++) {
      const a = pts[i], b = pts[(i + 1) % pts.length];
      s += a.x * b.y - b.x * a.y;
    }
    return s / 2;
  },
  /** Absolute polygon area via the shoelace formula. */
  polygonArea(pts) { return Math.abs(G.polygonSignedArea(pts)); },

  centroid(pts) {
    let x = 0, y = 0;
    for (const p of pts) { x += p.x; y += p.y; }
    const n = pts.length || 1;
    return { x: x / n, y: y / n };
  },

  /** Ray-casting point-in-polygon (boundary counts as inside within TOL). */
  pointInPolygon(p, poly) {
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const a = poly[i], b = poly[j];
      if (G.distPointSeg(p, a, b) <= G.TOL) return true;
      if ((a.y > p.y) !== (b.y > p.y)) {
        const xInt = (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x;
        if (p.x < xInt) inside = !inside;
      }
    }
    return inside;
  },

  closestPointOnSeg(p, a, b) {
    const ab = G.sub(b, a);
    const l2 = G.dot(ab, ab);
    if (l2 < G.EPS) return { x: a.x, y: a.y };
    const t = Util.clamp(G.dot(G.sub(p, a), ab) / l2, 0, 1);
    return { x: a.x + ab.x * t, y: a.y + ab.y * t };
  },
  distPointSeg(p, a, b) { return G.dist(p, G.closestPointOnSeg(p, a, b)); },

  /** True when segments ab and cd properly intersect (shared endpoints and
      collinear touching do not count — tolerance TOL). */
  segProperIntersect(a, b, c, d) {
    const d1 = G.cross(G.sub(b, a), G.sub(c, a));
    const d2 = G.cross(G.sub(b, a), G.sub(d, a));
    const d3 = G.cross(G.sub(d, c), G.sub(a, c));
    const d4 = G.cross(G.sub(d, c), G.sub(b, c));
    const t = G.TOL * Math.max(G.dist(a, b), G.dist(c, d), 1);
    return ((d1 > t && d2 < -t) || (d1 < -t && d2 > t)) &&
           ((d3 > t && d4 < -t) || (d3 < -t && d4 > t));
  },

  /** Rectangle polygon: centre (cx,cy), size sx along local x, sy along
      local y, rotated by angDeg. Returns 4 corners. */
  rectPoly(cx, cy, sx, sy, angDeg) {
    const a = G.d2r(angDeg || 0);
    const c = Math.cos(a), s = Math.sin(a);
    const hx = sx / 2, hy = sy / 2;
    const pts = [{ x: -hx, y: -hy }, { x: hx, y: -hy }, { x: hx, y: hy }, { x: -hx, y: hy }];
    return pts.map(p => ({ x: cx + p.x * c - p.y * s, y: cy + p.x * s + p.y * c }));
  },

  /** Parking-stall polygon: centre, width w across, length l along the axis
      whose direction is axisDeg (degrees, 0 = +x, 90 = +y/down). */
  stallPoly(cx, cy, w, l, axisDeg) {
    return G.rectPoly(cx, cy, w, l, axisDeg - 90);
  },

  bbox(pts) {
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for (const p of pts) {
      if (p.x < minX) minX = p.x;
      if (p.y < minY) minY = p.y;
      if (p.x > maxX) maxX = p.x;
      if (p.y > maxY) maxY = p.y;
    }
    return { minX, minY, maxX, maxY, w: maxX - minX, h: maxY - minY };
  },

  /** SAT overlap test for two CONVEX polygons. Returns true only when the
      penetration depth exceeds TOL (touching edges are not an overlap). */
  convexOverlap(A, B) {
    const polys = [A, B];
    for (let pi = 0; pi < 2; pi++) {
      const poly = polys[pi];
      for (let i = 0; i < poly.length; i++) {
        const a = poly[i], b = poly[(i + 1) % poly.length];
        const axis = G.norm(G.perp(G.sub(b, a)));
        let minA = Infinity, maxA = -Infinity, minB = Infinity, maxB = -Infinity;
        for (const p of A) { const d = G.dot(p, axis); if (d < minA) minA = d; if (d > maxA) maxA = d; }
        for (const p of B) { const d = G.dot(p, axis); if (d < minB) minB = d; if (d > maxB) maxB = d; }
        if (maxA - minB <= G.TOL || maxB - minA <= G.TOL) return false;
      }
    }
    return true;
  },

  /** Convex polygon fully inside a (possibly concave) polygon:
      every corner inside AND no proper edge intersections. */
  polyInsidePolygon(inner, outer) {
    for (const p of inner) if (!G.pointInPolygon(p, outer)) return false;
    for (let i = 0; i < inner.length; i++) {
      const a = inner[i], b = inner[(i + 1) % inner.length];
      for (let j = 0; j < outer.length; j++) {
        const c = outer[j], d = outer[(j + 1) % outer.length];
        if (G.segProperIntersect(a, b, c, d)) return false;
      }
    }
    return true;
  },

  /** Minimum distance between a convex polygon and a segment.
      (For convex shapes the minimum is attained at a vertex of one of
      them, so checking poly corners vs segment and segment endpoints vs
      poly edges is exact.) Returns 0 if they intersect. */
  polySegDist(poly, a, b) {
    let d = Infinity;
    for (let i = 0; i < poly.length; i++) {
      const p = poly[i], q = poly[(i + 1) % poly.length];
      if (G.segProperIntersect(p, q, a, b)) return 0;
      d = Math.min(d, G.distPointSeg(p, a, b));
      d = Math.min(d, G.distPointSeg(a, p, q), G.distPointSeg(b, p, q));
    }
    return d;
  },

  /** Clip polygon with the half-plane { p : dot(p − p0, n) ≥ 0 }.
      Sutherland–Hodgman single-plane clip. */
  clipHalfPlane(poly, p0, n) {
    const out = [];
    const side = p => G.dot(G.sub(p, p0), n);
    for (let i = 0; i < poly.length; i++) {
      const cur = poly[i], nxt = poly[(i + 1) % poly.length];
      const sc = side(cur), sn = side(nxt);
      if (sc >= -G.EPS) out.push(cur);
      if ((sc > G.EPS && sn < -G.EPS) || (sc < -G.EPS && sn > G.EPS)) {
        const t = sc / (sc - sn);
        out.push({ x: cur.x + (nxt.x - cur.x) * t, y: cur.y + (nxt.y - cur.y) * t });
      }
    }
    return out;
  },

  /** Clip a polygon against a CONVEX clip polygon (Sutherland–Hodgman:
      successive half-plane clips along the clip polygon's edges).
      Returns the intersection polygon ([] when disjoint). */
  convexClip(subject, clip) {
    let out = subject.slice();
    for (let i = 0; i < clip.length && out.length >= 3; i++) {
      const n = G.inwardNormal(clip, i);
      out = G.clipHalfPlane(out, clip[i], n);
    }
    return out.length >= 3 ? out : [];
  },

  /** Area of the intersection of two convex polygons (0 when disjoint). */
  convexOverlapArea(a, b) {
    const inter = G.convexClip(a, b);
    return inter.length >= 3 ? G.polygonArea(inter) : 0;
  },

  /** Inward normal of edge i of a polygon (unit vector pointing toward
      the polygon interior, robust to winding direction). */
  inwardNormal(poly, i) {
    const a = poly[i], b = poly[(i + 1) % poly.length];
    const mid = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
    let n = G.norm(G.perp(G.sub(b, a)));
    const c = G.centroid(poly);
    if (G.dot(n, G.sub(c, mid)) < 0) n = G.scale(n, -1);
    return n;
  },

  /** Inset a polygon by a per-edge distance using successive half-plane
      clips. Exact for convex polygons; conservative (may over-clip) for
      concave ones. Returns [] when the inset consumes the polygon. */
  insetPolygonPerEdge(poly, insets) {
    let out = poly.slice();
    for (let i = 0; i < poly.length; i++) {
      const s = insets[i] || 0;
      if (s <= 0) continue;
      const a = poly[i];
      const n = G.inwardNormal(poly, i);
      const p0 = { x: a.x + n.x * s, y: a.y + n.y * s };
      out = G.clipHalfPlane(out, p0, n);
      if (out.length < 3) return [];
    }
    return out;
  },

  /** Cast a ray p + t·dir (t > 0) against polygon edges; returns
      { t, edge } for the farthest exit intersection, or null. */
  rayPolygonExit(p, dir, poly) {
    let best = null;
    for (let i = 0; i < poly.length; i++) {
      const a = poly[i], b = poly[(i + 1) % poly.length];
      const v = G.sub(b, a);
      const denom = G.cross(dir, v);
      if (Math.abs(denom) < G.EPS) continue;
      const t = G.cross(G.sub(a, p), v) / denom;
      const u = G.cross(G.sub(a, p), dir) / denom;
      if (t > G.TOL && u >= -0.001 && u <= 1.001) {
        if (!best || t > best.t) best = { t, edge: i };
      }
    }
    return best;
  },

  /** First entry intersection of ray p + t·dir (t > 0) with a polygon,
      or Infinity when the ray misses it. */
  rayPolygonEnter(p, dir, poly) {
    let best = Infinity;
    for (let i = 0; i < poly.length; i++) {
      const a = poly[i], b = poly[(i + 1) % poly.length];
      const v = G.sub(b, a);
      const denom = G.cross(dir, v);
      if (Math.abs(denom) < G.EPS) continue;
      const t = G.cross(G.sub(a, p), v) / denom;
      const u = G.cross(G.sub(a, p), dir) / denom;
      if (t > G.TOL && u >= -0.001 && u <= 1.001 && t < best) best = t;
    }
    return best;
  },

  /** Rotate world → frame (frame x-axis at angDeg in world). */
  toFrame(p, angDeg) { return G.rot(p, -G.d2r(angDeg)); },
  fromFrame(p, angDeg) { return G.rot(p, G.d2r(angDeg)); },

  /**
   * Project WGS84 lat/lng vertices to local metres with a tangent-plane
   * (equirectangular) projection centred on the parcel — centimetre-level
   * accuracy at parcel scale. Returns { pts, centroid } with x east and
   * y south (north up on screen), translated to positive coordinates.
   */
  latLngToLocal(coords) {
    let lat0 = 0, lng0 = 0;
    for (const c of coords) { lat0 += c.lat; lng0 += c.lng; }
    lat0 /= coords.length; lng0 /= coords.length;
    const mPerLat = 110574;
    const mPerLng = 111320 * Math.cos(G.d2r(lat0));
    let pts = coords.map(c => ({
      x: (c.lng - lng0) * mPerLng,
      y: (lat0 - c.lat) * mPerLat          // north points up (−y)
    }));
    const bb = G.bbox(pts);
    pts = pts.map(p => ({
      x: Math.round((p.x - bb.minX) * 100) / 100,
      y: Math.round((p.y - bb.minY) * 100) / 100
    }));
    return { pts, centroid: { lat: lat0, lng: lng0 } };
  }
};

/* ═══════════════════════ 3. State ═══════════════════════ */

const State = {
  s: null,               // live application state (see defaults())
  undoStack: [],
  redoStack: [],
  UNDO_LIMIT: 60,

  /** Demo project per the specification (§19). */
  defaults() {
    return {
      version: 1,
      project: { name: 'Demo Project — Commercial Site', number: 'PRJ-001', designer: '', client: '', lat: null, lng: null },
      land: { mode: 'rect', width: 60, depth: 45, rotation: 0, northAngle: 0, polygon: [] },
      roads: [{
        id: 'road1', edge: 2, name: 'Main Street', width: 20, classification: 'collector',
        minWidth: 12, designation: 'front', accessAllowed: true, maxEntrances: 2
      }],
      accessPoints: [{ id: 'ap1', edge: 2, t: 0.5, width: 6, type: 'both' }],
      accessRules: { minCornerDist: 5, minSpacing: 12, throatDepth: 6, maxDeadEnd: 30 },
      buildings: [{
        id: 'b1', name: 'Building 1', width: 24, depth: 18, x: 30, y: 15.5,
        rotation: 0, locked: false, defMethod: 'wh', area: 432, aspect: 1.33
      }],
      selectedBuilding: 0,
      setbacks: { front: 6, rear: 3, left: 3, right: 3, road: 0, wallClearance: 0, bldgToParking: 0.5, bldgToAisle: 0.5 },
      sidewalks: { building: 2, street: 0, landscapeBuffer: 0 },
      parking: {
        stallW: 2.5, stallL: 5.5, angle: '90', rowType: 'auto',
        aisleOneWay90: 6, aisleTwoWay90: 6, aisle60: 4.5, aisle45: 3.5, aisle30: 3.5,
        aisleParallel: 3.5, aisleMain: 6, fireRoute: 6
      },
      accessible: { stallW: 3.6, stallL: 5.5, aisleW: 1.5, ratioPer: 25, min: 1 },
      ev: { pct: 5, clearance: 0.5 },
      frontage: { enabled: true },
      demand: {
        method: 'perArea', sqmPerSpace: 30, gfa: 1200, floors: 3, autoGfa: false,
        per100: 3.3, gfa2: 1200, fixed: 40,
        mixed: [
          { use: 'Bank branch', area: 600, ratio: 30 },
          { use: 'Office', area: 600, ratio: 40 }
        ],
        sharedReductionPct: 0
      },
      regs: { maxCoveragePct: 60, minLandscapePct: 0 },
      optimization: {
        tryO0: true, tryO90: true, tryLongest: true, tryBuilding: true,
        tryCustom: false, customAngle: 0,
        weights: { count: 10, compliance: 8, circulation: 6, accessProx: 4, landscape: 3, simplicity: 3 },
        option: 'B'
      },
      zones: [],        // { id, type:'noparking'|'landscape'|'crossing', x, y, w, h, angle }
      dimensions: [],   // { id, a:{x,y}, b:{x,y} }
      manual: { removed: [], typeOverrides: {}, added: [], rowShift: {}, rowDir: {} },
      view: {
        snap: true, grid: true,
        layers: {
          grid: true, roads: true, setbacks: true, sidewalks: true, landscape: true,
          aisles: true, parking: true, arrows: true, building: true, zones: true,
          dimensions: true, labels: true
        }
      }
    };
  },

  init() {
    let restored = null;
    try {
      const raw = localStorage.getItem('app.autosave');
      if (raw) restored = JSON.parse(raw);
    } catch (e) { restored = null; }
    this.s = this.migrate(restored) || this.defaults();
  },

  /** Merge a possibly older/partial saved state over fresh defaults so new
      fields always exist. Values are type-checked against the defaults:
      arrays must be arrays, numeric fields must be finite numbers — junk
      keeps the default instead of poisoning the state. Returns null when
      input is unusable. */
  migrate(saved) {
    if (!saved || typeof saved !== 'object' || Array.isArray(saved) || !saved.land) return null;
    const merge = (base, over) => {
      for (const k in over) {
        const bv = base[k], ov = over[k];
        if (Array.isArray(bv)) {
          if (Array.isArray(ov)) base[k] = ov;
        } else if (bv !== null && typeof bv === 'object') {
          if (ov !== null && typeof ov === 'object' && !Array.isArray(ov)) merge(bv, ov);
        } else if (typeof bv === 'number') {
          const n = Number(ov);
          if (isFinite(n)) base[k] = n;
        } else if (typeof bv === 'boolean') {
          base[k] = !!ov;
        } else if (ov === null || ['string', 'number', 'boolean'].includes(typeof ov)) {
          base[k] = ov;   // string / nullable metadata fields
        }
      }
      return base;
    };
    try { return merge(this.defaults(), saved); } catch (e) { return null; }
  },

  autosave: Util.debounce(function () {
    try { localStorage.setItem('app.autosave', JSON.stringify(State.s)); } catch (e) { /* storage full/blocked */ }
  }, 800),

  /** Read a dotted path like "parking.stallW". */
  get(path) {
    return path.split('.').reduce((o, k) => (o == null ? undefined : o[k]), this.s);
  },
  set(path, value) {
    const keys = path.split('.');
    const last = keys.pop();
    const target = keys.reduce((o, k) => (o[k] == null ? (o[k] = {}) : o[k]), this.s);
    target[last] = value;
  },

  pushUndo() {
    this.undoStack.push(JSON.stringify(this.s));
    if (this.undoStack.length > this.UNDO_LIMIT) this.undoStack.shift();
    this.redoStack.length = 0;
    UI.updateUndoButtons();
  },
  undo() {
    if (!this.undoStack.length) return false;
    this.redoStack.push(JSON.stringify(this.s));
    this.s = this.migrate(JSON.parse(this.undoStack.pop()));
    return true;
  },
  redo() {
    if (!this.redoStack.length) return false;
    this.undoStack.push(JSON.stringify(this.s));
    this.s = this.migrate(JSON.parse(this.redoStack.pop()));
    return true;
  },

  /* ── Presets (§16) — stored in localStorage only ── */

  PRESET_KEY: 'app.presets',
  /** Fields captured by a municipality preset. */
  presetPaths: [
    'demand.method', 'demand.sqmPerSpace', 'demand.per100', 'demand.fixed',
    'parking.stallW', 'parking.stallL', 'parking.angle',
    'parking.aisleOneWay90', 'parking.aisleTwoWay90', 'parking.aisle60', 'parking.aisle45',
    'parking.aisle30', 'parking.aisleParallel', 'parking.aisleMain', 'parking.fireRoute',
    'accessible.stallW', 'accessible.stallL', 'accessible.aisleW', 'accessible.ratioPer', 'accessible.min',
    'ev.pct', 'ev.clearance',
    'setbacks.front', 'setbacks.rear', 'setbacks.left', 'setbacks.right', 'setbacks.road',
    'regs.maxCoveragePct', 'regs.minLandscapePct',
    'accessRules.minCornerDist', 'accessRules.minSpacing',
    'sidewalks.landscapeBuffer', 'frontage.enabled'
  ],

  /** Built-in example preset. Values are editable defaults, NOT verified
      legal requirements (see disclaimer). */
  builtinPresets() {
    return {
      'Saudi Municipality — Custom Project Preset': {
        'demand.method': 'perArea', 'demand.sqmPerSpace': 30,
        'parking.stallW': 2.5, 'parking.stallL': 5.5, 'parking.angle': '90',
        'parking.aisleOneWay90': 6, 'parking.aisleTwoWay90': 6, 'parking.aisle60': 4.5,
        'parking.aisle45': 3.5, 'parking.aisle30': 3.5, 'parking.aisleParallel': 3.5,
        'parking.aisleMain': 6, 'parking.fireRoute': 6,
        'accessible.stallW': 3.6, 'accessible.stallL': 5.5, 'accessible.aisleW': 1.5,
        'accessible.ratioPer': 25, 'accessible.min': 1,
        'ev.pct': 5, 'ev.clearance': 0.5,
        'setbacks.front': 6, 'setbacks.rear': 3, 'setbacks.left': 3, 'setbacks.right': 3, 'setbacks.road': 0,
        'regs.maxCoveragePct': 60, 'regs.minLandscapePct': 10,
        'accessRules.minCornerDist': 5, 'accessRules.minSpacing': 12,
        'sidewalks.landscapeBuffer': 0, 'frontage.enabled': true
      }
    };
  },

  loadPresetStore() {
    let store = {};
    try { store = JSON.parse(localStorage.getItem(this.PRESET_KEY) || '{}'); } catch (e) { store = {}; }
    const builtin = this.builtinPresets();
    for (const k in builtin) if (!(k in store)) store[k] = builtin[k];
    return store;
  },
  savePresetStore(store) {
    try { localStorage.setItem(this.PRESET_KEY, JSON.stringify(store)); } catch (e) { /* ignore */ }
  },
  capturePreset() {
    const p = {};
    for (const path of this.presetPaths) p[path] = this.get(path);
    return p;
  },
  applyPreset(preset) {
    for (const path in preset) {
      if (this.presetPaths.includes(path)) this.set(path, preset[path]);
    }
  }
};

/* ═══════════════════════ 4. Demand ═══════════════════════ */

const Demand = {
  /** Total building footprint area (m²) across all buildings. */
  footprintArea(s) {
    return s.buildings.reduce((sum, b) => sum + b.width * b.depth, 0);
  },

  /** Compute required parking spaces for the active method. Returns
      { required, baseRequired, gfa, breakdown[] }. */
  compute(s) {
    const d = s.demand;
    let base = 0, gfa = 0;
    const breakdown = [];
    if (d.method === 'perArea') {
      gfa = d.autoGfa ? this.footprintArea(s) * Math.max(1, d.floors) : d.gfa;
      base = d.sqmPerSpace > 0 ? Math.ceil(gfa / d.sqmPerSpace) : 0;
      breakdown.push({ use: 'Building GFA', area: gfa, req: base });
    } else if (d.method === 'per100') {
      gfa = d.gfa2;
      base = Math.ceil(gfa / 100 * d.per100);
      breakdown.push({ use: 'Building GFA', area: gfa, req: base });
    } else if (d.method === 'fixed') {
      base = Math.max(0, Math.round(d.fixed));
      breakdown.push({ use: 'Fixed requirement', area: null, req: base });
    } else if (d.method === 'mixed') {
      for (const row of d.mixed) {
        const req = row.ratio > 0 ? Math.ceil(row.area / row.ratio) : 0;
        gfa += row.area;
        base += req;
        breakdown.push({ use: row.use, area: row.area, req });
      }
    }
    const reduction = Util.clamp(d.sharedReductionPct || 0, 0, 90) / 100;
    const required = Math.max(0, Math.ceil(base * (1 - reduction)));
    return { required, baseRequired: base, gfa, breakdown };
  }
};

/* ═══════════════════════ 6. Rules — compliance engine ═══════════════════════ */

const Rules = {
  /** Advisory hard floors used only to sanity-check the user's own
      editable standards (the editable value remains authoritative). */
  FLOORS: { twoWayAisle: 6.0, oneWayAisle: 3.5, stallW: 2.3, stallL: 5.0 },

  /**
   * Evaluate everything and return { items:[{level,msg}], status } where
   * level ∈ 'bad' | 'warn' and status ∈ 'ok' | 'warn' | 'bad'.
   */
  evaluate(s, ctx, layout, demand) {
    const items = [];
    const bad = m => items.push({ level: 'bad', msg: m });
    const warn = m => items.push({ level: 'warn', msg: m });

    /* Roads */
    for (const r of s.roads) {
      if (r.width < r.minWidth) {
        bad(`Road “${r.name}” width ${Util.fmt(r.width)} m is below its minimum permitted width of ${Util.fmt(r.minWidth)} m.`);
      }
    }
    if (!s.roads.length) warn('No road is defined — the site has no street frontage.');

    /* Building placement */
    for (const info of ctx.buildingInfos) {
      for (const v of info.violations) bad(v);
    }

    /* Coverage */
    const coverage = ctx.landArea > 0 ? Demand.footprintArea(s) / ctx.landArea * 100 : 0;
    if (coverage > s.regs.maxCoveragePct) {
      bad(`Building coverage ${Util.fmt(coverage, 1)}% exceeds the maximum permitted ${Util.fmt(s.regs.maxCoveragePct)}%.`);
    }

    /* Parking counts */
    const st = layout ? layout.stats : null;
    if (st) {
      if (st.total < demand.required) {
        bad(`Generated parking (${st.total}) is below the required count (${demand.required}) — deficit of ${demand.required - st.total} space(s).`);
      }
      const needAcc = st.total > 0 ? Math.max(s.accessible.min, Math.ceil(st.total / Math.max(1, s.accessible.ratioPer))) : 0;
      if (st.accessible < needAcc) {
        bad(`Accessible parking is insufficient: ${st.accessible} provided, ${needAcc} required (1 per ${s.accessible.ratioPer}, minimum ${s.accessible.min}).`);
      }
      const needEV = Math.ceil(st.total * Util.clamp(s.ev.pct, 0, 100) / 100);
      if (st.ev < needEV) {
        warn(`EV parking is insufficient: ${st.ev} provided, ${needEV} required (${Util.fmt(s.ev.pct)}% of provided spaces).`);
      }
      if (st.droppedDisconnected > 0) {
        warn(`Vehicle circulation is disconnected: ${st.droppedDisconnected} potential stall(s) were removed because their aisle cannot be reached from an entrance.`);
      }
      for (const de of st.deadEnds) {
        warn(`Dead-end aisle of ${Util.fmt(de, 1)} m exceeds the maximum permitted ${Util.fmt(s.accessRules.maxDeadEnd)} m.`);
      }
      if (st.crossingConflicts > 0) {
        warn(`${st.crossingConflicts} parking stall(s) intersect a pedestrian crossing.`);
      }
      if (st.frontParallelAisles > 0) {
        warn(`${st.frontParallelAisles} internal drive aisle(s) run parallel and adjacent to the street frontage — municipal practice expects stalls fronting the street, with side/rear parking accessed from a side lane.`);
      }
    }

    /* Aisle standards sanity checks */
    if (s.parking.aisleTwoWay90 < this.FLOORS.twoWayAisle) {
      warn(`Two-way 90° aisle width ${Util.fmt(s.parking.aisleTwoWay90)} m is below the customary ${Util.fmt(this.FLOORS.twoWayAisle)} m standard.`);
    }
    if (s.parking.aisleOneWay90 < this.FLOORS.oneWayAisle || s.parking.aisle45 < this.FLOORS.oneWayAisle ||
        s.parking.aisle60 < this.FLOORS.oneWayAisle || s.parking.aisle30 < this.FLOORS.oneWayAisle) {
      warn(`A one-way aisle width is below the customary ${Util.fmt(this.FLOORS.oneWayAisle)} m minimum.`);
    }
    if (s.parking.stallW < this.FLOORS.stallW || s.parking.stallL < this.FLOORS.stallL) {
      warn(`Stall size ${Util.fmt(s.parking.stallW)} × ${Util.fmt(s.parking.stallL)} m is below customary minimums (${this.FLOORS.stallW} × ${this.FLOORS.stallL} m).`);
    }
    if (s.parking.aisleMain < s.parking.fireRoute) {
      warn(`Main circulation aisle (${Util.fmt(s.parking.aisleMain)} m) is narrower than the fire-truck route requirement (${Util.fmt(s.parking.fireRoute)} m) — the fire route may be obstructed.`);
    }

    /* Access points */
    if (!s.accessPoints.length && demand.required > 0) {
      warn('No vehicle entrance/exit is defined — parking aisles cannot be reached from the road network.');
    }
    for (const ap of ctx.apInfos) {
      if (ap.road && !ap.road.accessAllowed) {
        bad(`Access point on ${ap.edgeName} lies on a road where vehicle access is not allowed.`);
      }
      if (!ap.road) {
        bad(`Access point on ${ap.edgeName} lies on an edge with no road.`);
      }
      if (ap.cornerDist < s.accessRules.minCornerDist) {
        bad(`Entrance on ${ap.edgeName} is ${Util.fmt(ap.cornerDist, 1)} m from the site corner — closer than the minimum ${Util.fmt(s.accessRules.minCornerDist)} m.`);
      }
    }
    // spacing between access points on the same edge
    const byEdge = {};
    for (const ap of ctx.apInfos) (byEdge[ap.edge] = byEdge[ap.edge] || []).push(ap);
    for (const e in byEdge) {
      const list = byEdge[e].slice().sort((a, b) => a.distAlong - b.distAlong);
      for (let i = 1; i < list.length; i++) {
        const gap = list[i].distAlong - list[i - 1].distAlong;
        if (gap < s.accessRules.minSpacing) {
          warn(`Two access points on ${list[i].edgeName} are ${Util.fmt(gap, 1)} m apart — closer than the minimum spacing of ${Util.fmt(s.accessRules.minSpacing)} m.`);
        }
      }
      const road = list[0].road;
      if (road && list.length > road.maxEntrances) {
        warn(`${list.length} access points on ${list[0].edgeName} exceed the ${road.maxEntrances} entrance(s) permitted for “${road.name}”.`);
      }
    }

    /* Landscape */
    if (st && ctx.landArea > 0) {
      const lsPct = st.landscapeArea / ctx.landArea * 100;
      if (lsPct < s.regs.minLandscapePct) {
        warn(`Landscape area ${Util.fmt(lsPct, 1)}% is below the required minimum of ${Util.fmt(s.regs.minLandscapePct)}%.`);
      }
    }

    const status = items.some(i => i.level === 'bad') ? 'bad' : (items.length ? 'warn' : 'ok');
    return { items, status };
  }
};

/* ═══════════════════════ 5. Generator ═══════════════════════ */

const Generator = {

  /* ─────────────────────────────────────────────────────────────────
     Site context: everything derived from the inputs that the layout
     generator and rules engine need (land polygon, edge roles and
     insets, building obstacles, access throats and spine aisles).
     ───────────────────────────────────────────────────────────────── */
  buildContext(s) {
    const ctx = {
      valid: false, landPoly: null, edges: [], insets: [], buildingInsets: [],
      landArea: 0, frontEdgeIdx: 2, buildingInfos: [], obstaclesStall: [],
      obstaclesAisle: [], throats: [], spines: [], apInfos: [], entrancePt: null,
      sidewalkArea: 0, edgeRoles: []
    };

    /* Land polygon */
    let poly = null;
    if (s.land.mode === 'rect') {
      const W = Math.max(1, s.land.width), D = Math.max(1, s.land.depth);
      poly = [{ x: 0, y: 0 }, { x: W, y: 0 }, { x: W, y: D }, { x: 0, y: D }];
      if (s.land.rotation) {
        const c = { x: W / 2, y: D / 2 };
        poly = poly.map(p => G.rotAround(p, c, G.d2r(s.land.rotation)));
      }
    } else {
      if (s.land.polygon.length >= 3) poly = s.land.polygon.map(p => ({ x: p.x, y: p.y }));
    }
    if (!poly || G.polygonArea(poly) < 1) return ctx;   // nothing to plan yet

    ctx.landPoly = poly;
    ctx.landArea = G.polygonArea(poly);
    ctx.edges = poly.map((a, i) => {
      const b = poly[(i + 1) % poly.length];
      return { idx: i, a, b, len: G.dist(a, b), name: this.edgeName(s, i, poly.length) };
    });

    /* Front edge & roles */
    const validRoads = s.roads.filter(r => r.edge >= 0 && r.edge < poly.length);
    const frontRoad = validRoads.find(r => r.designation === 'front') || validRoads[0] || null;
    ctx.frontEdgeIdx = frontRoad ? frontRoad.edge : (s.land.mode === 'rect' ? 2 : 0);
    ctx.edgeRoles = poly.map((_, i) => this.edgeRole(s, ctx, i, validRoads));
    ctx.roadEdges = validRoads.map(r => r.edge);

    /* Per-edge insets */
    const sb = s.setbacks, sw = s.sidewalks;
    const roleSetback = role => role === 'front' ? sb.front
      : role === 'rear' ? sb.rear
      : role === 'left' ? sb.left
      : role === 'right' ? sb.right
      : Math.max(sb.left, sb.right);
    ctx.insets = poly.map((_, i) => {
      const role = ctx.edgeRoles[i];
      const road = validRoads.find(r => r.edge === i);
      let inset = roleSetback(role);
      inset = Math.max(inset, road ? Math.max(sb.road, sw.street) : sb.wallClearance);
      inset = Math.max(inset, sw.landscapeBuffer);
      return inset;
    });
    ctx.buildingInsets = poly.map((_, i) => {
      const role = ctx.edgeRoles[i];
      const road = validRoads.find(r => r.edge === i);
      return Math.max(roleSetback(role), road ? sb.road : 0, sw.landscapeBuffer);
    });

    /* Buildings: footprint, sidewalk ring, obstacles, compliance */
    for (let bi = 0; bi < s.buildings.length; bi++) {
      const b = s.buildings[bi];
      const bpoly = G.rectPoly(b.x, b.y, b.width, b.depth, b.rotation);
      const swPoly = G.rectPoly(b.x, b.y, b.width + 2 * sw.building, b.depth + 2 * sw.building, b.rotation);
      const obSt = G.rectPoly(b.x, b.y,
        b.width + 2 * (sw.building + sb.bldgToParking),
        b.depth + 2 * (sw.building + sb.bldgToParking), b.rotation);
      const obAi = G.rectPoly(b.x, b.y,
        b.width + 2 * (sw.building + sb.bldgToAisle),
        b.depth + 2 * (sw.building + sb.bldgToAisle), b.rotation);
      const violations = [];
      if (!G.polyInsidePolygon(bpoly, poly)) {
        violations.push(`${b.name} extends beyond the land boundary.`);
      } else {
        for (const e of ctx.edges) {
          const need = ctx.buildingInsets[e.idx];
          if (need > 0) {
            const d = G.polySegDist(bpoly, e.a, e.b);
            if (d < need - G.TOL) {
              violations.push(`${b.name} violates the ${ctx.edgeRoles[e.idx]} setback on the ${e.name} — ${Util.fmt(d, 2)} m provided, ${Util.fmt(need)} m required.`);
            }
          }
        }
      }
      for (let bj = 0; bj < bi; bj++) {
        if (G.convexOverlap(bpoly, ctx.buildingInfos[bj].poly)) {
          violations.push(`${b.name} overlaps ${s.buildings[bj].name}.`);
        }
      }
      ctx.buildingInfos.push({ b, poly: bpoly, swPoly, obSt, obAi, violations });
      ctx.obstaclesStall.push(obSt);
      ctx.obstaclesAisle.push(obAi);
      ctx.sidewalkArea += G.polygonArea(swPoly) - G.polygonArea(bpoly);
    }

    /* User zones that block parking */
    for (const z of s.zones) {
      if (z.type === 'noparking' || z.type === 'landscape') {
        const zp = G.rectPoly(z.x + z.w / 2, z.y + z.h / 2, z.w, z.h, z.angle || 0);
        ctx.obstaclesStall.push(zp);
        ctx.obstaclesAisle.push(zp);
      }
    }

    /* Access points: throat clear zones + spine aisles */
    for (const ap of s.accessPoints) {
      if (ap.edge < 0 || ap.edge >= poly.length) continue;
      const e = ctx.edges[ap.edge];
      const tang = G.norm(G.sub(e.b, e.a));
      const n = G.inwardNormal(poly, ap.edge);
      const pos = { x: e.a.x + tang.x * e.len * ap.t, y: e.a.y + tang.y * e.len * ap.t };
      const road = validRoads.find(r => r.edge === ap.edge) || null;
      const distAlong = e.len * ap.t;
      const cornerDist = Math.min(distAlong, e.len - distAlong) - ap.width / 2;
      ctx.apInfos.push({ ap, edge: ap.edge, edgeName: e.name, pos, tang, normal: n, road, distAlong, cornerDist });

      const td = Math.max(0, s.accessRules.throatDepth);
      if (td > 0) {
        const hw = ap.width / 2 + 0.5;
        ctx.throats.push([
          G.add(pos, G.scale(tang, -hw)), G.add(pos, G.scale(tang, hw)),
          G.add(G.add(pos, G.scale(tang, hw)), G.scale(n, td)),
          G.add(G.add(pos, G.scale(tang, -hw)), G.scale(n, td))
        ]);
      }
      const ws = Math.max(s.parking.aisleMain, ap.width);
      const exit = G.rayPolygonExit(pos, n, poly);
      const boundaryL = Math.max(0, exit ? exit.t - (ctx.insets[exit.edge] || 0) : 0);
      let L = boundaryL;
      /* the spine stops at the first obstacle in its path (e.g. the building) */
      let trimOb = -1;
      for (let oi = 0; oi < ctx.obstaclesAisle.length; oi++) {
        const tEnter = G.rayPolygonEnter(pos, n, ctx.obstaclesAisle[oi]);
        if (tEnter < L) { L = tEnter; trimOb = oi; }
      }
      L = Math.max(0, L);
      if (L > 0.5) {
        const hw2 = ws / 2;
        ctx.spines.push({
          apId: ap.id,
          poly: [
            G.add(pos, G.scale(tang, -hw2)), G.add(pos, G.scale(tang, hw2)),
            G.add(G.add(pos, G.scale(tang, hw2)), G.scale(n, L)),
            G.add(G.add(pos, G.scale(tang, -hw2)), G.scale(n, L))
          ],
          pos, normal: n, len: L, width: ws
        });
      }
      /* When an obstacle blocks the straight run SHORTLY after the entrance
         (building right in front of it — not even one parking module fits
         before the obstruction), cars must turn: add a T-bar aisle across
         the spine end so circulation can route around. A longer spine
         reaches the normal aisle network and needs no T-bar. */
      if (L < boundaryL - 0.5 && L > 0.5 && L < ws + s.parking.stallL + 1) {
        const barEnd = L, barStart = Math.max(0.2, L - ws);
        const thick = barEnd - barStart;
        if (thick > 1.5) {
          const cEnd = G.add(pos, G.scale(n, (barStart + barEnd) / 2));
          const lateral = dirV => {
            const ex = G.rayPolygonExit(cEnd, dirV, poly);
            let t = ex ? ex.t - 0.4 : 0;
            for (const ob of ctx.obstaclesAisle) {
              const te = G.rayPolygonEnter(cEnd, dirV, ob);
              if (te < t) t = te;
            }
            return Math.max(0, t);
          };
          let tl = lateral(G.scale(tang, -1));
          let tr = lateral(tang);
          /* The turn always runs to ONE side only (the wider strip beside
             the blocking obstacle) plus a side lane past it — an internal
             road running parallel to the public street is never built;
             the frontage belongs to stalls or landscape. */
          if (trimOb >= 0) {
            const proj = p => G.dot(p, tang);
            const obPts = ctx.obstaclesAisle[trimOb];
            let obLo = Infinity, obHi = -Infinity, laLo = Infinity, laHi = -Infinity;
            for (const p of obPts) { const v = proj(p); if (v < obLo) obLo = v; if (v > obHi) obHi = v; }
            for (const p of poly) { const v = proj(p); if (v < laLo) laLo = v; if (v > laHi) laHi = v; }
            const sideDir = (obLo - laLo) >= (laHi - obHi) ? -1 : 1;
            if (sideDir < 0) tr = Math.min(tr, ws / 2 + 1); else tl = Math.min(tl, ws / 2 + 1);
            /* side lane hugging the obstacle, from the T-bar toward the rear */
            const p0v = proj(pos);
            let laneOff = sideDir < 0 ? obLo - ws / 2 - 0.2 : obHi + ws / 2 + 0.2;
            laneOff = Util.clamp(laneOff, laLo + ws / 2 + 0.3, laHi - ws / 2 - 0.3);
            const laneReach = sideDir < 0 ? (p0v - laneOff) : (laneOff - p0v);
            if (laneReach > 0 && laneReach < (sideDir < 0 ? tl : tr) + ws) {
              const laneFront = G.add(G.add(pos, G.scale(tang, laneOff - p0v)), G.scale(n, barStart));
              const lexit = G.rayPolygonExit(laneFront, n, poly);
              let laneLen = lexit ? lexit.t - (ctx.insets[lexit.edge] || 0) : 0;
              for (let oi = 0; oi < ctx.obstaclesAisle.length; oi++) {
                if (oi === trimOb) continue;
                const te = G.rayPolygonEnter(laneFront, n, ctx.obstaclesAisle[oi]);
                if (te < laneLen) laneLen = te;
              }
              if (laneLen > 2) {
                const hw3 = ws / 2;
                ctx.spines.push({
                  apId: ap.id + '_lane',
                  poly: [
                    G.add(laneFront, G.scale(tang, -hw3)), G.add(laneFront, G.scale(tang, hw3)),
                    G.add(G.add(laneFront, G.scale(tang, hw3)), G.scale(n, laneLen)),
                    G.add(G.add(laneFront, G.scale(tang, -hw3)), G.scale(n, laneLen))
                  ],
                  pos: laneFront, normal: n, len: laneLen, width: ws
                });
              }
            }
          }
          if (tl + tr > 3) {
            const p0 = G.add(cEnd, G.scale(tang, -tl));
            const half = G.scale(n, thick / 2);
            ctx.spines.push({
              apId: ap.id + '_bar', bar: true,
              poly: [
                G.sub(p0, half), G.sub(G.add(p0, G.scale(tang, tl + tr)), half),
                G.add(G.add(p0, G.scale(tang, tl + tr)), half), G.add(p0, half)
              ],
              pos: p0, normal: tang, len: tl + tr, width: thick
            });
          }
        }
      }
    }

    /* Principal building entrance: midpoint of the building side facing
       the front road (accessible stalls are clustered near it). */
    if (ctx.buildingInfos.length) {
      const fe = ctx.edges[ctx.frontEdgeIdx];
      const feMid = { x: (fe.a.x + fe.b.x) / 2, y: (fe.a.y + fe.b.y) / 2 };
      const bp = ctx.buildingInfos[0].poly;
      let best = null;
      for (let i = 0; i < 4; i++) {
        const m = { x: (bp[i].x + bp[(i + 1) % 4].x) / 2, y: (bp[i].y + bp[(i + 1) % 4].y) / 2 };
        const d = G.dist(m, feMid);
        if (!best || d < best.d) best = { d, m };
      }
      ctx.entrancePt = best.m;
    } else {
      ctx.entrancePt = G.centroid(poly);
    }

    /* Frontage stalls (municipal option): 90° stalls along every access-
       allowed road edge, nosed in and served DIRECTLY from the public
       street — they may sit inside the front setback strip, which is the
       point of the requirement. They keep clear of the building (incl.
       sidewalk + clearances), entrance throats, spines, zones and the
       side boundaries, and they become obstacles for the interior
       generator so nothing else is planned on top of them. */
    ctx.frontageStalls = [];
    if (s.frontage && s.frontage.enabled) {
      const fw = s.parking.stallW, fl = s.parking.stallL;
      const cornerClear = 2;
      for (const road of validRoads) {
        if (road.accessAllowed === false) continue;
        const e = ctx.edges[road.edge];
        if (!e || e.len < fw + 2 * cornerClear) continue;
        const tang = G.norm(G.sub(e.b, e.a));
        const n = G.inwardNormal(poly, e.idx);
        const axis = G.r2d(Math.atan2(n.y, n.x));
        let slot = 0;
        for (let d = cornerClear + fw / 2; d <= e.len - cornerClear - fw / 2 + 0.001; d += fw, slot++) {
          const outer = { x: e.a.x + tang.x * d, y: e.a.y + tang.y * d };
          const c = G.add(outer, G.scale(n, fl / 2));
          const stallP = G.stallPoly(c.x, c.y, fw, fl, axis);
          if (!G.polyInsidePolygon(stallP, poly)) continue;
          let bad = false;
          for (const ob of ctx.obstaclesStall) if (G.convexOverlap(stallP, ob)) { bad = true; break; }
          if (!bad) for (const th of ctx.throats) if (G.convexOverlap(stallP, th)) { bad = true; break; }
          if (!bad) for (const sp of ctx.spines) if (G.convexOverlap(stallP, sp.poly)) { bad = true; break; }
          if (!bad) for (const e2 of ctx.edges) {
            if (e2.idx === e.idx) continue;
            if (G.polySegDist(stallP, e2.a, e2.b) < 1.0) { bad = true; break; }
          }
          if (bad) continue;
          ctx.frontageStalls.push({
            id: 'f' + e.idx + '_' + slot, key: 'kf_' + e.idx + '_' + slot,
            cx: c.x, cy: c.y, poly: stallP, axisWorld: axis,
            band: -2, row: 0, slot, segId: null,
            type: 'regular', connected: true, frontage: true
          });
          ctx.obstaclesStall.push(stallP);
          ctx.obstaclesAisle.push(stallP);
        }
      }
    }

    ctx.valid = true;
    return ctx;
  },

  edgeName(s, i, n) {
    if (s.land.mode === 'rect' && n === 4) return ['north edge', 'east edge', 'south edge', 'west edge'][i];
    return 'edge ' + (i + 1);
  },

  edgeRole(s, ctx, i, validRoads) {
    const road = validRoads.find(r => r.edge === i);
    if (s.land.mode === 'rect') {
      const f = ctx.frontEdgeIdx;
      if (i === f) return 'front';
      if (i === (f + 2) % 4) return 'rear';
      if (i === (f + 1) % 4) return 'left';
      return 'right';
    }
    if (road) return road.designation === 'front' ? (i === ctx.frontEdgeIdx ? 'front' : 'side') : road.designation;
    return 'side';
  },

  /* ─────────────────────────────────────────────────────────────────
     Stall / aisle geometry parameters for a parking-angle key.
     pitch  — centre-to-centre slot spacing along the row
     depth  — row depth measured perpendicular to the aisle
     axis   — stall length-axis angle (frame degrees) for the row
              below the aisle; the row above uses the mirrored angle.
     ───────────────────────────────────────────────────────────────── */
  stallGeom(s, key) {
    const w = s.parking.stallW, l = s.parking.stallL, p = s.parking;
    if (key === 'parallel') {
      return { key, pitch: l + 0.6, depth: w, axisLow: 0, axisHigh: 0, aisle: p.aisleParallel, oneWay: true, label: 'Parallel' };
    }
    const deg = parseFloat(key);
    if (deg === 90) {
      /* single-loaded 90° rows run one-way; double-loaded rows two-way */
      const single = p.rowType === 'single';
      return {
        key, pitch: w, depth: l, axisLow: 90, axisHigh: -90,
        aisle: single ? p.aisleOneWay90 : p.aisleTwoWay90,
        oneWay: single, label: '90°'
      };
    }
    const rad = G.d2r(deg);
    const pitch = w / Math.sin(rad);
    const depth = l * Math.sin(rad) + w * Math.cos(rad);
    const aisle = deg === 60 ? p.aisle60 : deg === 45 ? p.aisle45 : p.aisle30;
    return { key, pitch, depth, axisLow: deg, axisHigh: -deg, aisle, oneWay: true, label: deg + '°' };
  },

  /* ─────────────────────────────────────────────────────────────────
     Generate one candidate layout.
       orient     — row direction in world degrees
       stallKey   — '90' | '60' | '45' | '30' | 'parallel'
       flip       — pack bands starting from the opposite side
       extraInset — additional landscape inset on every boundary edge

     If rows end up disconnected from every entrance spine, retry with
     perimeter CONNECTOR aisles at the row ends (the standard loop-road
     solution) and keep whichever variant yields more connected stalls.
     ───────────────────────────────────────────────────────────────── */
  generateCandidate(s, ctx, orient, stallKey, flip, extraInset, forceConnectors, anchor, connWhich) {
    if (forceConnectors !== undefined) {
      return this.generateCandidateRaw(s, ctx, orient, stallKey, flip, extraInset, forceConnectors, anchor, connWhich);
    }
    let best = this.generateCandidateRaw(s, ctx, orient, stallKey, flip, extraInset, false, anchor);
    if (best.stats.droppedDisconnected > 0 && ctx.spines.length) {
      /* try loop connectors on one side only first — the other side keeps
         its stalls — then on both sides; keep whichever parks the most */
      for (const which of ['left', 'right', 'both']) {
        const alt = this.generateCandidateRaw(s, ctx, orient, stallKey, flip, extraInset, true, anchor, which);
        if (alt.stats.total > best.stats.total) best = alt;
      }
    }
    return best;
  },

  generateCandidateRaw(s, ctx, orient, stallKey, flip, extraInset, useConnectors, anchor, connWhich) {
    const geom = this.stallGeom(s, stallKey);
    const insets = ctx.insets.map(v => v + extraInset);
    const insetPoly = G.insetPolygonPerEdge(ctx.landPoly, insets);
    const layout = {
      meta: {
        orient, stallKey, flip, extraInset, geom,
        useConnectors: !!useConnectors, anchor: anchor == null ? null : anchor,
        connWhich: connWhich || 'both'
      },
      stalls: [], aisles: [], bands: [], stats: null, score: 0
    };
    if (insetPoly.length < 3) { layout.stats = this.emptyStats(s, ctx, layout); return layout; }

    const toF = p => G.toFrame(p, orient);
    const fromF = p => G.fromFrame(p, orient);
    const insetF = insetPoly.map(toF);
    const bb = G.bbox(insetF);

    /* Frame-space obstacle boxes for aisle blocking */
    const aisleBlockers = ctx.obstaclesAisle.map(ob => G.bbox(ob.map(toF)));

    /* Row-type policy */
    const rt = s.parking.rowType;
    const allowDouble = rt !== 'single';
    const allowSingle = rt !== 'double';
    const modD = geom.depth * 2 + geom.aisle;
    const modS = geom.depth + geom.aisle;

    /* Build bands along frame-y */
    let cursor = flip ? bb.maxY : bb.minY;
    const dir = flip ? -1 : 1;
    /* Anchored packing: shift the band grid so one drive aisle is centred
       on the access point's axis — the natural layout for narrow sites
       where the entrance aisle doubles as the main parking aisle. */
    if (anchor != null && !flip) {
      const modBase = allowDouble ? modD : modS;
      if (modBase > 0.1) {
        let off = ((anchor - geom.aisle / 2 - geom.depth) - bb.minY) % modBase;
        if (off < 0) off += modBase;
        if (off > 0.05 && off < modBase - 0.05) cursor = bb.minY + off;
      }
    }
    let bi = 0;
    while (true) {
      const remaining = flip ? cursor - bb.minY : bb.maxY - cursor;
      let double = false;
      if (allowDouble && remaining >= modD - 0.001) double = true;
      else if (allowSingle && remaining >= modS - 0.001) double = false;
      else break;
      const mod = double ? modD : modS;
      const b0 = dir > 0 ? cursor : cursor - mod;
      const b1 = dir > 0 ? cursor + mod : cursor;
      const rows = [];
      let aY0, aY1;
      /* serpentine flow: odd bands mirror the stall lean so the alternating
         one-way aisle directions match the angled geometry (no-op for 90°) */
      const mir = (bi % 2 === 0) ? 1 : -1;
      if (double) {
        aY0 = b0 + geom.depth; aY1 = b1 - geom.depth;
        rows.push({ yc: b0 + geom.depth / 2, axis: mir * geom.axisHigh, side: 0 });  // above aisle
        rows.push({ yc: b1 - geom.depth / 2, axis: mir * geom.axisLow, side: 1 });   // below aisle
      } else if (dir > 0) {
        aY0 = b0 + geom.depth; aY1 = b1;
        rows.push({ yc: b0 + geom.depth / 2, axis: mir * geom.axisHigh, side: 0 });
      } else {
        aY0 = b0; aY1 = b0 + geom.aisle;
        rows.push({ yc: b1 - geom.depth / 2, axis: mir * geom.axisLow, side: 1 });
      }
      layout.bands.push({ bi, b0, b1, aY0, aY1, rows, double });
      cursor += dir * mod;
      bi++;
    }

    /* Aisle segments per band (split around obstacles), then stalls */
    const spinePolys = ctx.spines.map(sp => sp.poly);
    const spineBBoxF = ctx.spines.map(sp => G.bbox(sp.poly.map(toF)));
    let segId = 0;

    /* Perimeter connector aisles at the row ends (loop circulation).
       Each strip is split around obstacles and clipped to the inset
       polygon; unusably short slivers are discarded. */
    let conn = null;
    if (useConnectors && layout.bands.length) {
      const wc = Math.max(s.parking.aisleMain, geom.aisle);
      const landF = ctx.landPoly.map(toF);
      const lbb = G.bbox(landF);
      conn = {
        wc,
        left: { x0: bb.minX, x1: bb.minX + wc },
        right: { x0: bb.maxX - wc, x1: bb.maxX },
        y0: Math.min(...layout.bands.map(b => b.b0)),
        y1: Math.max(...layout.bands.map(b => b.b1)),
        segs: [], landF
      };
      /* Extend connectors (as driveways, allowed through setbacks) toward
         spines / entrance T-bars that sit outside the band range, so the
         loop can actually reach the entrance. */
      for (const sp of ctx.spines) {
        const sbbF = G.bbox(sp.poly.map(toF));
        if (sbbF.minY > conn.y1 - 0.1) {
          conn.y1 = Math.min(lbb.maxY - 0.3, (sbbF.minY + sbbF.maxY) / 2);
        }
        if (sbbF.maxY < conn.y0 + 0.1) {
          conn.y0 = Math.max(lbb.minY + 0.3, (sbbF.minY + sbbF.maxY) / 2);
        }
      }
      const strips = connWhich === 'left' ? [conn.left]
        : connWhich === 'right' ? [conn.right]
        : [conn.left, conn.right];
      for (const strip of strips) {
        strip.alive = false;
        const blocked = [];
        for (const ob of aisleBlockers) {
          if (ob.maxX > strip.x0 + 0.05 && ob.minX < strip.x1 - 0.05) {
            blocked.push([ob.minY - 0.1, ob.maxY + 0.1]);
          }
        }
        blocked.sort((p, q) => p[0] - q[0]);
        const free = [];
        let y = conn.y0;
        for (const [f0, f1] of blocked) {
          if (f0 > y) free.push([y, Math.min(f0, conn.y1)]);
          y = Math.max(y, f1);
        }
        if (y < conn.y1) free.push([y, conn.y1]);
        for (const [f0, f1] of free) {
          if (f1 - f0 < Math.max(geom.aisle, 3)) continue;
          const rectF = [
            { x: strip.x0, y: f0 }, { x: strip.x0, y: f1 },
            { x: strip.x1, y: f1 }, { x: strip.x1, y: f0 }
          ];
          /* connectors are driveways: clipped to the land, not the insets,
             so they may legitimately cross a setback to reach the entrance */
          const clipped = G.convexClip(rectF, conn.landF);
          if (!clipped.length || G.polygonArea(clipped) < geom.aisle * 2) continue;
          const seg = {
            id: 's' + (segId++), band: -1, x0: strip.x0, x1: strip.x1,
            y0: f0, y1: f1, poly: clipped.map(fromF),
            oneWay: false, connected: false, spineHits: [], connector: true,
            stripX0: strip.x0, stripX1: strip.x1
          };
          conn.segs.push(seg);
          layout.aisles.push(seg);
          strip.alive = true;
        }
      }
      if (!conn.segs.length) conn = null;   // no usable connectors survived
    }

    for (const band of layout.bands) {
      /* blocked x-intervals across this band's aisle */
      const blocked = [];
      for (const ob of aisleBlockers) {
        if (ob.maxY > band.aY0 + 0.05 && ob.minY < band.aY1 - 0.05) {
          blocked.push([ob.minX - 0.1, ob.maxX + 0.1]);
        }
      }
      blocked.sort((a, b) => a[0] - b[0]);
      const free = [];
      let x = bb.minX;
      for (const [b0x, b1x] of blocked) {
        if (b0x > x) free.push([x, Math.min(b0x, bb.maxX)]);
        x = Math.max(x, b1x);
      }
      if (x < bb.maxX) free.push([x, bb.maxX]);

      band.segments = [];
      for (const [f0, f1] of free) {
        if (f1 - f0 < Math.max(geom.pitch, 3)) continue;
        const polyF = [{ x: f0, y: band.aY0 }, { x: f1, y: band.aY0 }, { x: f1, y: band.aY1 }, { x: f0, y: band.aY1 }];
        /* clip the aisle to the developable polygon so it never pokes
           outside rotated / irregular sites */
        const clippedF = G.convexClip(polyF, insetF);
        if (!clippedF.length) continue;
        const cbb = G.bbox(clippedF);
        const seg = {
          id: 's' + (segId++), band: band.bi, x0: f0, x1: f1,
          cx0: cbb.minX, cx1: cbb.maxX,
          y0: band.aY0, y1: band.aY1, poly: clippedF.map(fromF),
          oneWay: geom.oneWay, connected: false, spineHits: []
        };
        /* record where spines / connectors cross (used for dead-end lengths;
           actual connectivity is resolved later by a BFS over the aisle graph) */
        for (let si = 0; si < spinePolys.length; si++) {
          if (G.convexOverlap(seg.poly, spinePolys[si])) {
            seg.spineHits.push((spineBBoxF[si].minX + spineBBoxF[si].maxX) / 2);
          }
        }
        if (conn) {
          /* an end counts as covered only when a surviving connector
             sub-segment actually crosses this band's aisle there */
          const covers = cs => cs.y0 < band.aY1 - 0.05 && cs.y1 > band.aY0 + 0.05;
          if (seg.x0 <= conn.left.x1 + 0.1 &&
              conn.segs.some(cs => cs.stripX0 === conn.left.x0 && covers(cs))) {
            seg.spineHits.push(seg.x0);
          }
          if (seg.x1 >= conn.right.x0 - 0.1 &&
              conn.segs.some(cs => cs.stripX0 === conn.right.x0 && covers(cs))) {
            seg.spineHits.push(seg.x1);
          }
        }
        band.segments.push(seg);
        layout.aisles.push(seg);
      }

      /* stalls */
      for (let ri = 0; ri < band.rows.length; ri++) {
        const row = band.rows[ri];
        let slot = 0;
        for (let cx = bb.minX + geom.pitch / 2; cx <= bb.maxX - geom.pitch / 2 + 0.001; cx += geom.pitch, slot++) {
          /* parallel stalls lie along the row (length along x, width as depth) */
          const stallPolyF = geom.key === 'parallel'
            ? G.rectPoly(cx, row.yc, s.parking.stallL, s.parking.stallW, 0)
            : G.stallPoly(cx, row.yc, s.parking.stallW, s.parking.stallL, row.axis);
          if (conn) {
            /* only surviving connector strips exclude stalls */
            const sbb = G.bbox(stallPolyF);
            if ((conn.left.alive && sbb.maxX > conn.left.x0 && sbb.minX < conn.left.x1) ||
                (conn.right.alive && sbb.maxX > conn.right.x0 && sbb.minX < conn.right.x1)) continue;
          }
          const polyW = stallPolyF.map(fromF);
          const seg = (band.segments || []).find(sg => cx >= sg.x0 - 0.05 && cx <= sg.x1 + 0.05);
          if (!seg) continue;
          if (!this.validStall(polyW, ctx, insets, spinePolys)) continue;
          const cW = fromF({ x: cx, y: row.yc });
          layout.stalls.push({
            id: 'p' + band.bi + '_' + ri + '_' + slot,
            key: 'k' + Math.round(orient) + '_' + stallKey + '_' + (flip ? 1 : 0) + '_' + band.bi + '_' + ri + '_' + slot,
            cx: cW.x, cy: cW.y, fx: cx, fy: row.yc,
            poly: polyW, axisWorld: (geom.key === 'parallel' ? 0 : row.axis) + orient,
            band: band.bi, row: ri, slot, segId: seg.id,
            type: 'regular', connected: true
          });
        }
      }
    }

    /* Circulation: BFS over the aisle graph (spines seed it, touching
       aisles/connectors propagate), then drop stalls on unreachable
       aisles — unless there are no access points at all, in which case
       everything is kept and the rules engine warns. */
    this.connectAisles(layout.aisles, spinePolys);
    const segMap = {};
    for (const sg of layout.aisles) segMap[sg.id] = sg;
    let dropped = 0;
    if (spinePolys.length > 0) {
      const keep = [];
      for (const st of layout.stalls) {
        const sg = segMap[st.segId];
        st.connected = !!(sg && sg.connected);
        if (st.connected) keep.push(st); else dropped++;
      }
      layout.stalls = keep;
    }
    /* per-band world axis (used by manual row shifts and row labels) */
    layout.bandAxis = {};
    for (const band of layout.bands) layout.bandAxis[band.bi] = fromF({ x: 1, y: 0 });

    /* Dead-end aisles */
    const deadEnds = [];
    for (const seg of layout.aisles) {
      if (!seg.connected || !seg.spineHits.length || seg.connector) continue;
      const lo = seg.cx0 !== undefined ? seg.cx0 : seg.x0;
      const hi = seg.cx1 !== undefined ? seg.cx1 : seg.x1;
      const near = Math.min(...seg.spineHits) - lo;
      const far = hi - Math.max(...seg.spineHits);
      const worst = Math.max(near, far);
      if (worst > s.accessRules.maxDeadEnd) deadEnds.push(worst);
    }

    /* Frontage stalls (served directly from the street) join every layout */
    for (const fs of (ctx.frontageStalls || [])) layout.stalls.push(Object.assign({}, fs));

    this.assignSpecialStalls(layout, s, ctx);
    layout.stats = this.computeStats(s, ctx, layout, dropped, deadEnds);
    return layout;
  },

  /** Full validity test for one stall polygon (world coordinates). */
  validStall(polyW, ctx, insets, spinePolys) {
    if (!G.polyInsidePolygon(polyW, ctx.landPoly)) return false;
    for (const e of ctx.edges) {
      const need = insets[e.idx];
      if (need > 0 && G.polySegDist(polyW, e.a, e.b) < need - G.TOL) return false;
    }
    for (const ob of ctx.obstaclesStall) if (G.convexOverlap(polyW, ob)) return false;
    for (const th of ctx.throats) if (G.convexOverlap(polyW, th)) return false;
    for (const sp of spinePolys) if (G.convexOverlap(polyW, sp)) return false;
    return true;
  },

  /* ─────────────────────────────────────────────────────────────────
     Prune drive aisles that serve no stalls and are not needed to keep
     a stall-bearing aisle connected to an entrance — "aisles to
     nowhere" never appear on the drawing. Runs once on the displayed
     layout (not per candidate) so cost is negligible.
     ───────────────────────────────────────────────────────────────── */
  pruneAisles(layout, ctx) {
    if (!layout) return;
    const spinePolys = ctx.spines.map(sp => sp.poly);
    const used = new Set(layout.stalls.filter(st => st.segId).map(st => st.segId));
    let segs = layout.aisles.slice();

    /* with no entrances there is no network to preserve — empty aisles go */
    if (!spinePolys.length) {
      layout.aisles = segs.filter(sg => used.has(sg.id));
      return;
    }

    /** All stall-bearing segments still reachable from a spine in `list`? */
    const stillOk = list => {
      const present = new Set(list.map(sg => sg.id));
      for (const id of used) if (!present.has(id)) return false;
      const connected = new Set();
      const queue = [];
      for (const sg of list) {
        if (spinePolys.some(sp => G.convexOverlap(sg.poly, sp))) { connected.add(sg.id); queue.push(sg); }
      }
      while (queue.length) {
        const cur = queue.pop();
        for (const o of list) {
          if (!connected.has(o.id) && G.convexOverlap(cur.poly, o.poly)) { connected.add(o.id); queue.push(o); }
        }
      }
      for (const id of used) if (!connected.has(id)) return false;
      return true;
    };

    let changed = true;
    while (changed) {
      changed = false;
      for (let i = 0; i < segs.length; i++) {
        if (used.has(segs[i].id)) continue;
        const test = segs.slice(0, i).concat(segs.slice(i + 1));
        if (stillOk(test)) { segs = test; changed = true; break; }
      }
    }
    layout.aisles = segs;
  },

  /** Breadth-first connectivity across the circulation network: aisle
      segments touching a spine are seeds; any segment overlapping a
      connected segment becomes connected. With no spines (no access
      points) everything counts as connected and the rules engine warns. */
  connectAisles(segs, spinePolys) {
    if (!spinePolys.length) { for (const sg of segs) sg.connected = true; return; }
    const queue = [];
    for (const sg of segs) {
      sg.connected = spinePolys.some(sp => G.convexOverlap(sg.poly, sp));
      if (sg.connected) queue.push(sg);
    }
    while (queue.length) {
      const cur = queue.pop();
      for (const other of segs) {
        if (!other.connected && G.convexOverlap(cur.poly, other.poly)) {
          other.connected = true;
          queue.push(other);
        }
      }
    }
  },

  /* ─────────────────────────────────────────────────────────────────
     Ring layout: building-hugging strips (south / north / west / east
     of the building), each filled with 90° rows parallel to its own
     long axis and an aisle placed against the building first — the
     classic layout for a centred building. Generated as additional
     candidates alongside the banded sweeps.
     ───────────────────────────────────────────────────────────────── */
  generateRing(s, ctx, orient, extraInset) {
    const geom = this.stallGeom(s, '90');
    const insets = ctx.insets.map(v => v + extraInset);
    const insetPoly = G.insetPolygonPerEdge(ctx.landPoly, insets);
    const layout = {
      meta: { mode: 'ring', orient, stallKey: '90', flip: false, extraInset, geom, useConnectors: false },
      stalls: [], aisles: [], bands: [], bandAxis: {}, stats: null, score: 0
    };
    if (insetPoly.length < 3 || !ctx.buildingInfos.length) {
      layout.stats = this.emptyStats(s, ctx, layout);
      return layout;
    }
    const toF = p => G.toFrame(p, orient);
    const fromF = p => G.fromFrame(p, orient);
    const insetF = insetPoly.map(toF);
    const bb = G.bbox(insetF);
    const obb = G.bbox(ctx.buildingInfos[0].obAi.map(toF));
    const spinePolys = ctx.spines.map(sp => sp.poly);
    const aisleBlockers = ctx.obstaclesAisle.map(ob => G.bbox(ob.map(toF)));
    const depth = geom.depth, pitch = geom.pitch, aw = s.parking.aisleTwoWay90;
    let segId = 0, bi = 0;
    const placed = [];   // frame bboxes of accepted stalls (corner collisions)

    /** Split interval [a0,a1] around blocker intervals. */
    const freeIntervals = (a0, a1, blocked) => {
      blocked.sort((p, q) => p[0] - q[0]);
      const out = [];
      let x = a0;
      for (const [b0, b1] of blocked) {
        if (b0 > x) out.push([x, Math.min(b0, a1)]);
        x = Math.max(x, b1);
      }
      if (x < a1) out.push([x, a1]);
      return out.filter(iv => iv[1] - iv[0] >= Math.max(pitch, 3));
    };

    /** Frame-space rectangles of every aisle laid so far — stalls must
        never overlap a drive aisle (e.g. W/E columns ending at the S/N
        perimeter aisles). */
    const aisleRectsF = [];

    /**
     * Fill one strip. horiz=true → rows run along frame-x, packing along
     * frame-y away from the building (dir ±1) starting at `start`.
     * span = [s0,s1] across the rows (aisles); stalls are additionally
     * limited to [st0,st1] so they stop at neighbouring strips' aisles.
     */
    const fillStrip = (horiz, start, dir, lim, s0, s1, st0, st1) => {
      if (st0 === undefined) st0 = s0;
      if (st1 === undefined) st1 = s1;
      let cursor = start;
      let firstAisle = null;
      const avail = () => dir > 0 ? lim - cursor : cursor - lim;
      let guard = 0;
      let modulesPlaced = 0;
      while (guard++ < 20) {
        let rows = [];   // row centre offsets (along packing axis) facing the aisle at [aA0,aA1]
        let aA0, aA1;
        if (modulesPlaced === 0) {
          if (avail() < aw + depth - 0.001) break;
          /* aisle against the building, one row beyond it */
          aA0 = dir > 0 ? cursor : cursor - aw;
          aA1 = dir > 0 ? cursor + aw : cursor;
          rows.push(dir > 0 ? aA1 + depth / 2 : aA0 - depth / 2);
          cursor += dir * (aw + depth);
        } else {
          if (avail() < 2 * depth + aw - 0.001) break;
          /* back-to-back row, aisle, outer row */
          const r1 = cursor + dir * depth / 2;
          aA0 = dir > 0 ? cursor + depth : cursor - depth - aw;
          aA1 = aA0 + aw;
          rows.push(r1, cursor + dir * (depth + aw + depth / 2));
          cursor += dir * (2 * depth + aw);
        }
        /* aisle segments (split around blockers) */
        const blocked = [];
        for (const ob of aisleBlockers) {
          const lo = horiz ? ob.minY : ob.minX, hi = horiz ? ob.maxY : ob.maxX;
          const plo = horiz ? ob.minX : ob.minY, phi = horiz ? ob.maxX : ob.maxY;
          if (hi > aA0 + 0.05 && lo < aA1 - 0.05) blocked.push([plo - 0.1, phi + 0.1]);
        }
        const segs = [];
        for (const [f0, f1] of freeIntervals(s0, s1, blocked)) {
          const rect = horiz
            ? [{ x: f0, y: aA0 }, { x: f1, y: aA0 }, { x: f1, y: aA1 }, { x: f0, y: aA1 }]
            : [{ x: aA0, y: f0 }, { x: aA0, y: f1 }, { x: aA1, y: f1 }, { x: aA1, y: f0 }];
          const clipped = G.convexClip(rect, insetF);
          if (!clipped.length) continue;
          const seg = {
            id: 'r' + (segId++), band: bi, x0: f0, x1: f1, y0: aA0, y1: aA1,
            poly: clipped.map(fromF), oneWay: false, connected: true, spineHits: []
          };
          segs.push(seg);
          layout.aisles.push(seg);
          aisleRectsF.push(horiz
            ? { minX: f0, maxX: f1, minY: aA0, maxY: aA1 }
            : { minX: aA0, maxX: aA1, minY: f0, maxY: f1 });
        }
        if (firstAisle === null) firstAisle = [aA0, aA1];
        /* stalls */
        rows.forEach((rc, ri) => {
          let slot = 0;
          for (let c = st0 + pitch / 2; c <= st1 - pitch / 2 + 0.001; c += pitch, slot++) {
            const seg2 = segs.find(sg => c >= sg.x0 - 0.05 && c <= sg.x1 + 0.05);
            if (!seg2) continue;
            const fx = horiz ? c : rc, fy = horiz ? rc : c;
            const axis = horiz ? 90 : 0;
            const stallPolyF = G.stallPoly(fx, fy, s.parking.stallW, s.parking.stallL, axis);
            const sbbF = G.bbox(stallPolyF);
            let collide = false;
            for (const pb of placed) {
              if (sbbF.minX < pb.maxX - G.TOL && sbbF.maxX > pb.minX + G.TOL &&
                  sbbF.minY < pb.maxY - G.TOL && sbbF.maxY > pb.minY + G.TOL) { collide = true; break; }
            }
            /* never park inside any drive aisle (own aisle only touches) */
            if (!collide) {
              for (const ar of aisleRectsF) {
                if (sbbF.minX < ar.maxX - G.TOL && sbbF.maxX > ar.minX + G.TOL &&
                    sbbF.minY < ar.maxY - G.TOL && sbbF.maxY > ar.minY + G.TOL) { collide = true; break; }
              }
            }
            if (collide) continue;
            const polyW = stallPolyF.map(fromF);
            if (!this.validStall(polyW, ctx, insets, spinePolys)) continue;
            placed.push(sbbF);
            const cW = fromF({ x: fx, y: fy });
            layout.stalls.push({
              id: 'r' + bi + '_' + ri + '_' + slot,
              key: 'kr' + Math.round(orient) + '_' + bi + '_' + ri + '_' + slot,
              cx: cW.x, cy: cW.y, fx, fy, poly: polyW,
              axisWorld: axis + orient, band: bi, row: ri, slot, segId: seg2.id,
              type: 'regular', connected: true
            });
          }
        });
        layout.bands.push({ bi, b0: Math.min(start, cursor), b1: Math.max(start, cursor), double: modulesPlaced > 0, rows: [], segments: segs });
        layout.bandAxis[bi] = fromF(horiz ? { x: 1, y: 0 } : { x: 0, y: 1 });
        bi++;
        modulesPlaced++;
      }
      return firstAisle;
    };

    /* S and N strips take the full width. W/E strip AISLES run to the far
       edge of the first S/N aisle (so cars can loop around the building)
       but their STALL columns stop at the near edge — no parking inside
       the perimeter aisles. */
    const sAisle = fillStrip(true, obb.maxY, 1, bb.maxY, bb.minX, bb.maxX);
    const nAisle = fillStrip(true, obb.minY, -1, bb.minY, bb.minX, bb.maxX);
    const wLimS = sAisle ? sAisle[1] : bb.maxY;
    const wLimN = nAisle ? nAisle[0] : bb.minY;
    const stallLimS = sAisle ? sAisle[0] : bb.maxY;
    const stallLimN = nAisle ? nAisle[1] : bb.minY;
    fillStrip(false, obb.minX, -1, bb.minX, wLimN, wLimS, stallLimN, stallLimS);
    fillStrip(false, obb.maxX, 1, bb.maxX, wLimN, wLimS, stallLimN, stallLimS);

    this.connectAisles(layout.aisles, spinePolys);
    const segMap = {};
    for (const sg of layout.aisles) segMap[sg.id] = sg;
    let dropped = 0;
    if (spinePolys.length > 0) {
      const keep = [];
      for (const st of layout.stalls) {
        const sg = segMap[st.segId];
        st.connected = !!(sg && sg.connected);
        if (st.connected) keep.push(st); else dropped++;
      }
      layout.stalls = keep;
    }
    for (const fs of (ctx.frontageStalls || [])) layout.stalls.push(Object.assign({}, fs));
    this.assignSpecialStalls(layout, s, ctx);
    layout.stats = this.computeStats(s, ctx, layout, dropped, []);
    return layout;
  },

  /** Reserve accessible stalls (with shared access aisles) near the
      principal entrance, then tag EV stalls (§10.8–10.9). */
  assignSpecialStalls(layout, s, ctx) {
    const ent = ctx.entrancePt || { x: 0, y: 0 };
    const usable = () => layout.stalls.filter(st => st.type !== 'accessAisle');

    /* Accessible requirement is based on the provided count, which shrinks
       as slots are consumed for access aisles — iterate to a fixed point. */
    let needAcc = 0;
    for (let iter = 0; iter < 4; iter++) {
      const N = usable().length;
      const target = N > 0 ? Math.max(s.accessible.min, Math.ceil(N / Math.max(1, s.accessible.ratioPer))) : 0;
      if (target === needAcc) break;
      needAcc = target;
      /* reset previous pass */
      for (const st of layout.stalls) if (st.type === 'accessible' || st.type === 'accessAisle') st.type = 'regular';
      let made = 0;
      const byDist = layout.stalls.slice().sort((a, b) =>
        G.dist({ x: a.cx, y: a.cy }, ent) - G.dist({ x: b.cx, y: b.cy }, ent));
      const findNeighbor = (st, delta) => layout.stalls.find(o =>
        o.band === st.band && o.row === st.row && o.slot === st.slot + delta);
      for (const st of byDist) {
        if (made >= needAcc) break;
        if (st.type !== 'regular') continue;
        /* prefer sharing an existing access aisle */
        const left = findNeighbor(st, -1), right = findNeighbor(st, 1);
        if ((left && left.type === 'accessAisle') || (right && right.type === 'accessAisle')) {
          st.type = 'accessible'; made++; continue;
        }
        const donor = (right && right.type === 'regular') ? right : (left && left.type === 'regular') ? left : null;
        st.type = 'accessible'; made++;
        if (donor && s.accessible.aisleW > 0) donor.type = 'accessAisle';
      }
    }

    /* EV stalls: percentage of provided spaces, nearest to the entrance.
       When a charger clearance is required, prefer row-end stalls (a free
       flank leaves room for the charging unit). */
    const provided = usable();
    const needEV = Math.ceil(provided.length * Util.clamp(s.ev.pct, 0, 100) / 100);
    const isRowEnd = st => {
      const l = layout.stalls.find(o => o.band === st.band && o.row === st.row && o.slot === st.slot - 1);
      const r = layout.stalls.find(o => o.band === st.band && o.row === st.row && o.slot === st.slot + 1);
      return !l || !r;
    };
    const candidates = provided.filter(st => st.type === 'regular')
      .sort((a, b) => {
        if (s.ev.clearance > 0) {
          const ea = isRowEnd(a) ? 0 : 1, eb = isRowEnd(b) ? 0 : 1;
          if (ea !== eb) return ea - eb;
        }
        return G.dist({ x: a.cx, y: a.cy }, ent) - G.dist({ x: b.cx, y: b.cy }, ent);
      });
    for (let i = 0; i < Math.min(needEV, candidates.length); i++) candidates[i].type = 'ev';
  },

  emptyStats(s, ctx, layout) {
    return {
      total: 0, regular: 0, accessible: 0, ev: 0, accessAisles: 0, manual: 0,
      paved: 0, landscapeArea: Math.max(0, ctx.landArea - Demand.footprintArea(s) - ctx.sidewalkArea),
      droppedDisconnected: 0, deadEnds: [], fragments: 0, bandCount: 0,
      crossingConflicts: 0, avgAccDist: 0, frontParallelAisles: 0
    };
  },

  /** Count drive aisles that run parallel AND hard against a street
      frontage (closer than one stall length, so no stall row can front
      the street there) — the "inner road duplicating the public street"
      pattern that municipal practice rejects. */
  countFrontParallelAisles(s, ctx, layout) {
    if (!ctx.roadEdges || !ctx.roadEdges.length) return 0;
    let count = 0;
    for (const seg of layout.aisles) {
      let dir = null, best = 0;
      for (let i = 0; i < seg.poly.length; i++) {
        const a = seg.poly[i], b = seg.poly[(i + 1) % seg.poly.length];
        const l = G.dist(a, b);
        if (l > best) { best = l; dir = G.norm(G.sub(b, a)); }
      }
      if (!dir) continue;
      for (const ei of ctx.roadEdges) {
        const e = ctx.edges[ei];
        if (!e) continue;
        const ed = G.norm(G.sub(e.b, e.a));
        if (Math.abs(G.cross(dir, ed)) > 0.26) continue;   // > ~15° → not parallel
        if (G.polySegDist(seg.poly, e.a, e.b) < s.parking.stallL - 0.5) { count++; break; }
      }
    }
    return count;
  },

  computeStats(s, ctx, layout, dropped, deadEnds) {
    const st = this.emptyStats(s, ctx, layout);
    st.droppedDisconnected = dropped;
    st.deadEnds = deadEnds;
    st.bandCount = layout.bands.length;
    st.fragments = layout.aisles.length;
    let stallArea = 0;
    const crossings = App.state().zones.filter(z => z.type === 'crossing')
      .map(z => G.rectPoly(z.x + z.w / 2, z.y + z.h / 2, z.w, z.h, z.angle || 0));
    let accDistSum = 0;
    for (const p of layout.stalls) {
      stallArea += G.polygonArea(p.poly);
      if (p.type === 'accessAisle') { st.accessAisles++; continue; }
      st.total++;
      if (p.type === 'accessible') { st.accessible++; accDistSum += G.dist({ x: p.cx, y: p.cy }, ctx.entrancePt); }
      else if (p.type === 'ev') st.ev++;
      else if (p.type === 'manual') { st.manual++; st.regular++; }
      else st.regular++;
      for (const c of crossings) if (G.convexOverlap(p.poly, c)) { st.crossingConflicts++; break; }
    }
    st.avgAccDist = st.accessible ? accDistSum / st.accessible : 0;
    /* aisles and spines deliberately overlap where they meet (ring corners,
       spine junctions) — subtract pairwise intersections so the paved KPI
       is not double-counted (residual error only at rare triple overlaps) */
    const pavePolys = layout.aisles.map(a => a.poly).concat(ctx.spines.map(sp => sp.poly));
    const paveBBs = pavePolys.map(p => G.bbox(p));
    let aisleArea = 0;
    for (const p of pavePolys) aisleArea += G.polygonArea(p);
    for (let i = 0; i < pavePolys.length; i++) {
      for (let j = i + 1; j < pavePolys.length; j++) {
        const a = paveBBs[i], b = paveBBs[j];
        if (a.minX >= b.maxX || b.minX >= a.maxX || a.minY >= b.maxY || b.minY >= a.maxY) continue;
        aisleArea -= G.convexOverlapArea(pavePolys[i], pavePolys[j]);
      }
    }
    st.paved = stallArea + Math.max(0, aisleArea);
    st.landscapeArea = Math.max(0, ctx.landArea - Demand.footprintArea(s) - ctx.sidewalkArea - st.paved);
    st.frontParallelAisles = this.countFrontParallelAisles(s, ctx, layout);
    return st;
  },

  /** Weighted score for comparing candidate layouts (§11). */
  score(s, ctx, layout, demand, wOverride) {
    const w = wOverride || s.optimization.weights;
    const st = layout.stats;
    const req = Math.max(1, demand.required);
    const complianceRatio = Math.min(1, st.total / req);
    let sc = 0;
    sc += w.count * st.total;
    sc += w.compliance * (complianceRatio * 20 + (st.total >= demand.required ? 20 : 0));
    sc -= w.circulation * (st.droppedDisconnected * 1.5 + st.deadEnds.length * 6);
    /* an inner road duplicating the public street is heavily penalised */
    sc -= 45 * (st.frontParallelAisles || 0);
    sc += w.accessProx * Math.max(0, 15 - st.avgAccDist / 4);
    /* landscape reward is capped so an empty site can never outscore a
       layout that actually parks cars */
    const lsPct = ctx.landArea > 0 ? st.landscapeArea / ctx.landArea * 100 : 0;
    sc += w.landscape * Math.min(50, lsPct);
    sc -= w.simplicity * (Math.max(0, st.fragments - st.bandCount) * 2 + st.bandCount * 0.5);
    layout.score = Math.round(sc * 10) / 10;
    layout.scoreDetail = { complianceRatio };
    return layout.score;
  },

  /** Candidate orientation angles in world degrees (§10.2). */
  orientationList(s, ctx) {
    const o = s.optimization;
    const list = [];
    if (o.tryO0) list.push(0);
    if (o.tryO90) list.push(90);
    if (o.tryLongest && ctx.edges.length) {
      let longest = ctx.edges[0];
      for (const e of ctx.edges) if (e.len > longest.len) longest = e;
      list.push(G.r2d(Math.atan2(longest.b.y - longest.a.y, longest.b.x - longest.a.x)));
    }
    if (o.tryBuilding && s.buildings.length) list.push(s.buildings[0].rotation);
    if (o.tryCustom) list.push(o.customAngle || 0);
    /* normalise to [0,180) and dedupe */
    const seen = new Set(), out = [];
    for (let a of list) {
      a = ((a % 180) + 180) % 180;
      const k = a.toFixed(1);
      if (!seen.has(k)) { seen.add(k); out.push(a); }
    }
    return out.length ? out : [0];
  },

  stallKeyList(s) {
    return s.parking.angle === 'auto' ? ['90', '60', '45', '30', 'parallel'] : [s.parking.angle];
  },

  /** Full optimization: build the candidate pools and pick options A/B/C. */
  optimize(s, ctx, demand) {
    if (!ctx.valid) return { A: null, B: null, C: null };
    const orients = this.orientationList(s, ctx);
    const keys = this.stallKeyList(s);
    const flips = [false, true];

    const wC = Object.assign({}, s.optimization.weights, {
      landscape: s.optimization.weights.landscape * 3 + 3
    });
    const pool0 = [], poolC = [];
    const extrasC = [1.0, 2.5];   // landscape buffers tried for option C
    for (const o of orients) for (const k of keys) for (const f of flips) {
      const cand = this.generateCandidate(s, ctx, o, k, f, 0);
      this.score(s, ctx, cand, demand);
      pool0.push(cand);
      for (const ex of extrasC) {
        const candC = this.generateCandidate(s, ctx, o, k, f, ex);
        this.score(s, ctx, candC, demand, wC);
        poolC.push(candC);
      }
    }

    /* Entrance-anchored banded candidates: shift the band grid so a drive
       aisle lines up with each access point's axis (the classic narrow-
       site layout where the entrance aisle is the main parking aisle).
       Only meaningful when the rows run parallel to the entrance spine. */
    for (const o of orients) for (const k of keys) for (const ai of ctx.apInfos) {
      const nF = G.toFrame(ai.normal, o);
      if (Math.abs(nF.x) < 0.9) continue;
      const anchor = G.toFrame(ai.pos, o).y;
      const cand = this.generateCandidate(s, ctx, o, k, false, 0, undefined, anchor);
      this.score(s, ctx, cand, demand);
      pool0.push(cand);
    }

    /* Ring (building-hugging) candidates for 90° parking */
    if (keys.includes('90') && s.buildings.length) {
      const ringOrients = [], seenR = new Set();
      for (const a of [0, s.buildings[0].rotation]) {
        const norm = ((a % 180) + 180) % 180;
        const key = norm.toFixed(1);
        if (!seenR.has(key)) { seenR.add(key); ringOrients.push(norm); }
      }
      for (const o of ringOrients) {
        const rc = this.generateRing(s, ctx, o, 0);
        this.score(s, ctx, rc, demand);
        pool0.push(rc);
        for (const ex of extrasC) {
          const rcC = this.generateRing(s, ctx, o, ex);
          this.score(s, ctx, rcC, demand, wC);
          poolC.push(rcC);
        }
      }
    }

    const byCount = pool0.slice().sort((a, b) => b.stats.total - a.stats.total || b.score - a.score);
    const byScore = pool0.slice().sort((a, b) => b.score - a.score);
    const byScoreC = poolC.slice().sort((a, b) => b.score - a.score);

    const A = byCount[0] || null;
    const B = byScore[0] || null;
    /* Option C: the most landscape that still meets the requirement; if
       nothing meets it, the most landscape among near-best stall counts
       (never a degenerate near-empty layout). */
    let C = null;
    if (poolC.length) {
      const meets = demand.required > 0 ? poolC.filter(c => c.stats.total >= demand.required) : [];
      let candidates = meets;
      if (!candidates.length) {
        const bestTotal = Math.max(...poolC.map(c => c.stats.total));
        candidates = poolC.filter(c => c.stats.total >= Math.ceil(bestTotal * 0.8));
      }
      C = candidates.length
        ? candidates.slice().sort((a, b) => b.stats.landscapeArea - a.stats.landscapeArea || b.score - a.score)[0]
        : byScoreC[0];
    }
    if (A) { A.option = 'A'; A.label = 'Maximum parking'; }
    if (B) { B.option = 'B'; B.label = 'Balanced layout'; }
    if (C) { C.option = 'C'; C.label = 'Maximum landscape'; }
    return { A, B, C };
  },

  /** Fast single-combination regeneration used as the drag preview.
      Also prunes useless aisles — this is the layout that gets drawn. */
  regenerateLike(s, ctx, demand, meta) {
    if (!ctx.valid) return null;
    const cand = meta.mode === 'ring'
      ? this.generateRing(s, ctx, meta.orient, meta.extraInset)
      : this.generateCandidate(s, ctx, meta.orient, meta.stallKey, meta.flip, meta.extraInset, meta.useConnectors, meta.anchor, meta.connWhich);
    this.pruneAisles(cand, ctx);
    cand.stats = this.computeStats(s, ctx, cand,
      cand.stats ? cand.stats.droppedDisconnected : 0,
      cand.stats ? cand.stats.deadEnds : []);
    this.score(s, ctx, cand, demand);
    return cand;
  },

  /* ─────────────────────────────────────────────────────────────────
     Manual edits (§13): row shifts, removals, type overrides and
     manually added stalls are re-applied on top of a generated layout.
     ───────────────────────────────────────────────────────────────── */
  applyManualEdits(layout, s, ctx) {
    if (!layout) return;
    const man = s.manual;

    /* Row shifts: move every stall of a band along the row axis, re-validate */
    const spinePolys = ctx.spines.map(sp => sp.poly);
    for (const bandKey in man.rowShift) {
      const shift = man.rowShift[bandKey];
      if (!shift) continue;
      const bi = parseInt(bandKey, 10);
      const dir = (layout.bandAxis && layout.bandAxis[bi]) || G.fromFrame({ x: 1, y: 0 }, layout.meta.orient);
      layout.stalls = layout.stalls.filter(st => {
        if (st.band !== bi) return true;
        st.cx += dir.x * shift; st.cy += dir.y * shift;
        st.poly = st.poly.map(p => ({ x: p.x + dir.x * shift, y: p.y + dir.y * shift }));
        return this.validStall(st.poly, ctx, ctx.insets.map(v => v + layout.meta.extraInset), spinePolys);
      });
    }

    /* Removals and type overrides (keyed by stable stall keys) */
    if (man.removed.length) {
      const rm = new Set(man.removed);
      layout.stalls = layout.stalls.filter(st => !rm.has(st.key));
    }
    for (const key in man.typeOverrides) {
      const st = layout.stalls.find(x => x.key === key);
      if (st) st.type = man.typeOverrides[key];
    }

    /* Manually added stalls: validate against site AND existing stalls */
    const added = [];
    for (const a of man.added) {
      const poly = G.stallPoly(a.x, a.y, s.parking.stallW, s.parking.stallL, a.axis);
      let ok = this.validStall(poly, ctx, ctx.insets, spinePolys);
      if (ok) {
        for (const st of layout.stalls) if (G.convexOverlap(poly, st.poly)) { ok = false; break; }
      }
      if (ok) {
        added.push({
          id: a.id, key: a.id, cx: a.x, cy: a.y, poly, axisWorld: a.axis,
          band: -1, row: -1, slot: -1, segId: null,
          /* type overrides are keyed by stall key (= id for manual stalls) */
          type: man.typeOverrides[a.id] || a.type || 'manual',
          connected: true, manual: true
        });
      }
    }
    layout.stalls = layout.stalls.concat(added);

    /* Refresh stats after edits */
    layout.stats = this.computeStats(s, ctx, layout,
      layout.stats ? layout.stats.droppedDisconnected : 0,
      layout.stats ? layout.stats.deadEnds : []);

    this.numberStalls(layout);
  },

  /** Sequential stall numbering (drawn on the plan and used by the CSV
      schedule). Access-aisle slots are not counted. */
  numberStalls(layout) {
    let n = 0;
    for (const st of layout.stalls) {
      st.num = st.type === 'accessAisle' ? null : ++n;
    }
  }
};

/* ═══════════════════════ 7. Renderer ═══════════════════════ */

const Renderer = {
  svg: null,
  vp: null,                 // world-space group (pan/zoom transform)
  layers: {},
  overlay: null,            // screen-space overlay (north arrow, scale bar)
  view: { k: 12, tx: 60, ty: 60 },   // k = pixels per metre
  LAYER_ORDER: ['grid', 'roads', 'land', 'setbacks', 'sidewalks', 'aisles', 'parking',
    'arrows', 'building', 'zones', 'access', 'dimensions', 'labels', 'temp', 'handles'],

  COLORS: {
    landFill: '#e8efdd', landStroke: '#2f3b2a',
    road: '#b9bec4', roadBad: '#e4b6b6', curb: '#6c757d',
    setbackLine: '#c05621', sidewalk: '#d9d9d2',
    aisle: '#d3d7dc', stall: '#fdfdfc', stallStroke: '#5b6570',
    accessible: '#cfe0f7', accessibleInk: '#1d4ed8', ev: '#d6f2de', evInk: '#15803d',
    accessAisle: '#eef3fb', building: '#e9dcb8', buildingStroke: '#8a6d3b',
    buildingBad: '#f6c8c8', buildingBadStroke: '#c0392b',
    zoneNo: '#e07b7b', zoneLs: '#9fcf8f', crossing: '#8d99a6',
    dim: '#8b3a86', text: '#333940', arrow: '#ffffff'
  },

  init(svgEl) {
    this.svg = svgEl;
    this.svg.textContent = '';
    const defs = Util.svgEl('defs', {}, this.svg);
    this.buildDefs(defs);
    this.vp = Util.svgEl('g', { id: 'vp' }, this.svg);
    for (const name of this.LAYER_ORDER) {
      const g = Util.svgEl('g', { id: 'ly_' + name }, this.vp);
      if (name === 'aisles' || name === 'parking' || name === 'arrows') {
        g.setAttribute('clip-path', 'url(#clipLand)');
      }
      this.layers[name] = g;
    }
    this.overlay = Util.svgEl('g', { id: 'overlay' }, this.svg);
    this.applyView();
  },

  buildDefs(defs) {
    /* setback hatch */
    const mk = (id, w, path, stroke, sw, bg) => {
      const p = Util.svgEl('pattern', {
        id, width: w, height: w, patternUnits: 'userSpaceOnUse'
      }, defs);
      if (bg) Util.svgEl('rect', { width: w, height: w, fill: bg }, p);
      Util.svgEl('path', { d: path, stroke, 'stroke-width': sw, fill: 'none' }, p);
      return p;
    };
    mk('patSetback', 1.6, 'M0,1.6 L1.6,0', '#b9a08a', 0.07);
    mk('patNoPark', 1.2, 'M0,1.2 L1.2,0 M0,0 L1.2,1.2', '#c0392b', 0.08);
    mk('patAccAisle', 0.9, 'M0,0.9 L0.9,0', '#5b8def', 0.09);
    /* grid patterns */
    const gm = Util.svgEl('pattern', { id: 'patGridMinor', width: 1, height: 1, patternUnits: 'userSpaceOnUse' }, defs);
    Util.svgEl('path', { d: 'M1,0 L0,0 0,1', fill: 'none', stroke: '#c3ccd6', 'stroke-width': 0.02 }, gm);
    const gM = Util.svgEl('pattern', { id: 'patGridMajor', width: 5, height: 5, patternUnits: 'userSpaceOnUse' }, defs);
    Util.svgEl('rect', { width: 5, height: 5, fill: 'url(#patGridMinor)' }, gM);
    Util.svgEl('path', { d: 'M5,0 L0,0 0,5', fill: 'none', stroke: '#a8b4c2', 'stroke-width': 0.05 }, gM);
    this.landClip = Util.svgEl('clipPath', { id: 'clipLand' }, defs);
  },

  /* ── view transforms ── */
  applyView() {
    const v = this.view;
    this.vp.setAttribute('transform', `matrix(${v.k},0,0,${v.k},${v.tx},${v.ty})`);
    const zoomEl = Util.el('statZoom');
    if (zoomEl) zoomEl.textContent = '1 m = ' + Util.fmt(v.k, 1) + ' px  (≈1:' + Math.round(96 / 25.4 * 1000 / v.k) + ')';
  },
  w2s(p) { return { x: p.x * this.view.k + this.view.tx, y: p.y * this.view.k + this.view.ty }; },
  s2w(p) { return { x: (p.x - this.view.tx) / this.view.k, y: (p.y - this.view.ty) / this.view.k }; },

  zoomAt(factor, sx, sy) {
    const v = this.view;
    const k2 = Util.clamp(v.k * factor, 0.5, 500);
    const w = this.s2w({ x: sx, y: sy });
    v.k = k2;
    v.tx = sx - w.x * k2;
    v.ty = sy - w.y * k2;
    this.applyView();
    this.drawOverlay();
  },

  zoomFit(ctx) {
    const rect = this.svg.getBoundingClientRect();
    if (!ctx || !ctx.valid) { this.view = { k: 10, tx: 40, ty: 40 }; this.applyView(); return; }
    let bb = G.bbox(ctx.landPoly);
    /* include roads */
    let margin = 4;
    for (const r of App.state().roads) margin = Math.max(margin, r.width + 4);
    bb = { minX: bb.minX - margin, minY: bb.minY - margin, maxX: bb.maxX + margin, maxY: bb.maxY + margin };
    const w = bb.maxX - bb.minX, h = bb.maxY - bb.minY;
    const k = Math.min((rect.width - 20) / w, (rect.height - 20) / h);
    this.view.k = Util.clamp(k, 0.5, 500);
    this.view.tx = (rect.width - w * this.view.k) / 2 - bb.minX * this.view.k;
    this.view.ty = (rect.height - h * this.view.k) / 2 - bb.minY * this.view.k;
    this.applyView();
    this.drawOverlay();
  },

  setScaleRatio(denom, ctx) {
    /* 1:denom on a 96-dpi screen: px per metre = 96/25.4 × 1000/denom */
    const k = 96 / 25.4 * 1000 / denom;
    const rect = this.svg.getBoundingClientRect();
    const c = ctx && ctx.valid ? G.centroid(ctx.landPoly) : { x: 0, y: 0 };
    this.view.k = Util.clamp(k, 0.5, 500);
    this.view.tx = rect.width / 2 - c.x * this.view.k;
    this.view.ty = rect.height / 2 - c.y * this.view.k;
    this.applyView();
    this.drawOverlay();
  },

  /* ── drawing helpers ── */
  pathOf(pts, close = true) {
    return pts.map((p, i) => (i ? 'L' : 'M') + p.x.toFixed(3) + ',' + p.y.toFixed(3)).join(' ') + (close ? ' Z' : '');
  },
  poly(parent, pts, attrs) {
    return Util.svgEl('path', Object.assign({ d: this.pathOf(pts) }, attrs), parent);
  },
  line(parent, a, b, attrs) {
    return Util.svgEl('line', Object.assign({ x1: a.x, y1: a.y, x2: b.x, y2: b.y }, attrs), parent);
  },
  wtext(parent, x, y, str, size, attrs) {
    const t = Util.svgEl('text', Object.assign({
      x, y, 'font-size': size, fill: this.COLORS.text,
      'font-family': 'Arial, Helvetica, sans-serif',
      'text-anchor': 'middle', 'dominant-baseline': 'middle'
    }, attrs || {}), parent);
    t.textContent = str;
    return t;
  },

  /** Architectural dimension between two world points, offset sideways. */
  dimension(parent, a, b, offset, opts) {
    const o = opts || {};
    const dirV = G.norm(G.sub(b, a));
    const n = G.perp(dirV);
    const a2 = G.add(a, G.scale(n, offset));
    const b2 = G.add(b, G.scale(n, offset));
    const col = o.color || this.COLORS.dim;
    const g = Util.svgEl('g', {}, parent);
    const sw = { stroke: col, 'stroke-width': 0.045, fill: 'none' };
    this.line(g, a, a2, sw);
    this.line(g, b, b2, sw);
    this.line(g, a2, b2, sw);
    /* oblique ticks */
    for (const p of [a2, b2]) {
      const t1 = G.add(p, G.add(G.scale(dirV, -0.35), G.scale(n, -0.35)));
      const t2 = G.add(p, G.add(G.scale(dirV, 0.35), G.scale(n, 0.35)));
      this.line(g, t1, t2, { stroke: col, 'stroke-width': 0.07 });
    }
    const mid = G.scale(G.add(a2, b2), 0.5);
    const txtPos = G.add(mid, G.scale(n, offset >= 0 ? 0.75 : -0.75));
    let ang = G.r2d(Math.atan2(b.y - a.y, b.x - a.x));
    if (ang > 90 || ang < -90) ang += 180;
    const label = o.label || (Util.fmt(G.dist(a, b), 2) + ' m');
    this.wtext(g, txtPos.x, txtPos.y, label, o.size || 1.1, {
      fill: col, transform: `rotate(${ang} ${txtPos.x} ${txtPos.y})`
    });
    return g;
  },

  arrowGlyph(parent, cx, cy, angDeg, len, color) {
    const g = Util.svgEl('g', { transform: `translate(${cx},${cy}) rotate(${angDeg})` }, parent);
    const h = len / 2;
    Util.svgEl('path', {
      d: `M ${-h},0 L ${h * 0.55},0 M ${h * 0.1},-0.45 L ${h * 0.7},0 L ${h * 0.1},0.45 Z`,
      stroke: color, 'stroke-width': 0.16, fill: color, 'stroke-linecap': 'round'
    }, g);
    return g;
  },

  clearLayers() {
    for (const n of this.LAYER_ORDER) this.layers[n].textContent = '';
  },

  /* ─────────────────────────────────────────────────────────────────
     Main scene render.
     ───────────────────────────────────────────────────────────────── */
  render(s, ctx, layout, demand) {
    this.clearLayers();
    const L = this.layers, C = this.COLORS, lay = s.view.layers;
    this.landClip.textContent = '';
    if (!ctx.valid) {
      this.drawPolyDraft();
      this.drawOverlay();
      return;
    }
    Util.svgEl('path', { d: this.pathOf(ctx.landPoly) }, this.landClip);
    const bb = G.bbox(ctx.landPoly);

    /* grid */
    if (lay.grid && s.view.grid) {
      const m = 40;
      Util.svgEl('rect', {
        x: Math.floor(bb.minX - m), y: Math.floor(bb.minY - m),
        width: Math.ceil(bb.w + 2 * m), height: Math.ceil(bb.h + 2 * m),
        fill: this.view.k >= 5 ? 'url(#patGridMajor)' : 'url(#patGridMinor)'
      }, L.grid);
    }

    /* roads */
    if (lay.roads) this.drawRoads(s, ctx);

    /* land */
    this.poly(L.land, ctx.landPoly, {
      fill: C.landFill, stroke: C.landStroke, 'stroke-width': 2,
      'vector-effect': 'non-scaling-stroke'
    });

    /* setback zones + inner boundary */
    if (lay.setbacks) {
      for (const e of ctx.edges) {
        const inset = ctx.insets[e.idx];
        if (inset <= 0) continue;
        const n = G.inwardNormal(ctx.landPoly, e.idx);
        this.poly(L.setbacks, [e.a, e.b, G.add(e.b, G.scale(n, inset)), G.add(e.a, G.scale(n, inset))],
          { fill: 'url(#patSetback)', 'fill-opacity': 0.85, stroke: 'none' });
      }
      const ip = G.insetPolygonPerEdge(ctx.landPoly, ctx.insets);
      if (ip.length >= 3) {
        this.poly(L.setbacks, ip, {
          fill: 'none', stroke: C.setbackLine, 'stroke-width': 1.2,
          'stroke-dasharray': '6 4', 'vector-effect': 'non-scaling-stroke'
        });
      }
    }

    /* street sidewalks */
    if (lay.sidewalks && s.sidewalks.street > 0) {
      for (const r of s.roads) {
        if (r.edge < 0 || r.edge >= ctx.edges.length) continue;
        const e = ctx.edges[r.edge];
        const n = G.inwardNormal(ctx.landPoly, e.idx);
        this.poly(L.sidewalks, [e.a, e.b, G.add(e.b, G.scale(n, s.sidewalks.street)), G.add(e.a, G.scale(n, s.sidewalks.street))],
          { fill: C.sidewalk, stroke: '#b9b9b0', 'stroke-width': 0.04 });
      }
    }

    /* aisles + spines */
    if (lay.aisles && layout) {
      for (const seg of layout.aisles) {
        this.poly(L.aisles, seg.poly, { fill: C.aisle, stroke: '#aeb4bb', 'stroke-width': 0.04, 'data-seg': seg.id });
      }
      for (const sp of ctx.spines) {
        this.poly(L.aisles, sp.poly, { fill: C.aisle, stroke: '#aeb4bb', 'stroke-width': 0.04 });
      }
    }

    /* stalls */
    if (lay.parking && layout) this.drawStalls(s, layout);

    /* circulation arrows */
    if (lay.arrows && layout) this.drawArrows(s, ctx, layout);

    /* buildings + sidewalk ring */
    if (lay.building) this.drawBuildings(s, ctx);

    /* user zones (landscape islands are gated by the Landscape layer,
       no-parking zones and crossings by the Zones layer) */
    if (lay.zones || lay.landscape) this.drawZones(s);

    /* access points */
    this.drawAccessPoints(s, ctx);

    /* dimensions */
    if (lay.dimensions) this.drawDimensions(s, ctx);

    /* labels */
    if (lay.labels && layout) this.drawRowLabels(layout);

    this.drawPolyDraft();
    this.drawSelection(s, ctx, layout);
    this.drawOverlay();
  },

  drawRoads(s, ctx) {
    const L = this.layers, C = this.COLORS;
    for (const r of s.roads) {
      if (r.edge < 0 || r.edge >= ctx.edges.length) continue;
      const e = ctx.edges[r.edge];
      const out = G.scale(G.inwardNormal(ctx.landPoly, e.idx), -1);
      const bad = r.width < r.minWidth;
      const q = [e.a, e.b, G.add(e.b, G.scale(out, r.width)), G.add(e.a, G.scale(out, r.width))];
      this.poly(L.roads, q, { fill: bad ? C.roadBad : C.road, stroke: 'none' });
      /* centreline */
      const m0 = G.add(e.a, G.scale(out, r.width / 2));
      const m1 = G.add(e.b, G.scale(out, r.width / 2));
      this.line(L.roads, m0, m1, { stroke: '#f5f5f0', 'stroke-width': 0.15, 'stroke-dasharray': '2.5 2' });
      /* curb along the plot line */
      this.line(L.roads, e.a, e.b, { stroke: C.curb, 'stroke-width': 1.6, 'vector-effect': 'non-scaling-stroke' });
      /* label */
      const mid = G.scale(G.add(m0, m1), 0.5);
      let ang = G.r2d(Math.atan2(e.b.y - e.a.y, e.b.x - e.a.x));
      if (ang > 90 || ang < -90) ang += 180;
      this.wtext(L.roads, mid.x, mid.y, `${r.name}  (${Util.fmt(r.width)} m${bad ? ' — BELOW MIN ' + Util.fmt(r.minWidth) + ' m' : ''})`,
        Math.min(2, r.width * 0.22), {
          fill: bad ? '#8f2323' : '#5d646c', 'font-weight': bad ? '700' : '400',
          transform: `rotate(${ang} ${mid.x} ${mid.y})`
        });
    }
  },

  drawStalls(s, layout) {
    const L = this.layers, C = this.COLORS;
    for (const st of layout.stalls) {
      let fill = C.stall, extra = null;
      if (st.type === 'accessible') fill = C.accessible;
      else if (st.type === 'ev') fill = C.ev;
      else if (st.type === 'accessAisle') fill = C.accessAisle;
      const el = this.poly(L.parking, st.poly, {
        fill, stroke: C.stallStroke, 'stroke-width': 0.05,
        'data-stall': st.key
      });
      if (st.manual || st.type === 'manual') el.setAttribute('stroke-dasharray', '0.3 0.18');
      /* symbol + stall number: the number sits toward the stall's far end
         when a symbol occupies the centre, and reads across the stall */
      const hasSymbol = st.type === 'accessible' || st.type === 'ev';
      if (st.type === 'accessAisle') {
        this.poly(L.parking, st.poly, { fill: 'url(#patAccAisle)', stroke: 'none', 'pointer-events': 'none' });
      } else if (st.type === 'accessible') {
        this.wtext(L.parking, st.cx, st.cy, '♿', 1.6, { fill: C.accessibleInk, 'pointer-events': 'none' });
      } else if (st.type === 'ev') {
        this.wtext(L.parking, st.cx, st.cy, 'EV', 1.0, { fill: C.evInk, 'font-weight': '700', 'pointer-events': 'none' });
      }
      if (st.num != null && s.view.layers.labels) {
        let rot = (st.axisWorld || 90) - 90;
        rot = ((rot % 360) + 360) % 360;
        if (rot > 90 && rot < 270) rot -= 180;
        const ax = G.d2r(st.axisWorld || 90);
        const off = hasSymbol ? 1.7 : 0;
        const nx = st.cx + Math.cos(ax) * off, ny = st.cy + Math.sin(ax) * off;
        this.wtext(L.parking, nx, ny, String(st.num), 0.85, {
          fill: '#6b7280', 'font-weight': '600', 'pointer-events': 'none',
          transform: `rotate(${rot} ${nx} ${ny})`
        });
      }
    }
  },

  drawArrows(s, ctx, layout) {
    const L = this.layers, C = this.COLORS;
    for (const seg of layout.aisles) {
      const a = G.scale(G.add(seg.poly[0], seg.poly[3]), 0.5);
      const b = G.scale(G.add(seg.poly[1], seg.poly[2]), 0.5);
      const len = G.dist(a, b);
      if (len < 4) continue;
      const ang = G.r2d(Math.atan2(b.y - a.y, b.x - a.x));
      const dirFlip = (s.manual.rowDir[String(seg.band)] ? -1 : 1) * (seg.band % 2 === 0 ? 1 : -1);
      const n = Math.max(1, Math.floor(len / 9));
      for (let i = 1; i <= n; i++) {
        const t = i / (n + 1);
        const p = { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
        if (seg.oneWay) {
          this.arrowGlyph(L.arrows, p.x, p.y, ang + (dirFlip < 0 ? 180 : 0), 2.4, C.arrow);
        } else {
          const off = G.scale(G.norm(G.perp(G.sub(b, a))), 0.75);
          this.arrowGlyph(L.arrows, p.x + off.x, p.y + off.y, ang, 2.2, C.arrow);
          this.arrowGlyph(L.arrows, p.x - off.x, p.y - off.y, ang + 180, 2.2, C.arrow);
        }
      }
    }
    for (const sp of ctx.spines) {
      const a = sp.pos;
      const b = G.add(sp.pos, G.scale(sp.normal, sp.len));
      const ang = G.r2d(Math.atan2(b.y - a.y, b.x - a.x));
      const n = Math.max(1, Math.floor(sp.len / 9));
      const off = G.scale(G.norm(G.perp(sp.normal)), 0.75);
      for (let i = 1; i <= n; i++) {
        const t = i / (n + 1);
        const p = { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
        this.arrowGlyph(L.arrows, p.x + off.x, p.y + off.y, ang, 2.4, C.arrow);
        this.arrowGlyph(L.arrows, p.x - off.x, p.y - off.y, ang + 180, 2.4, C.arrow);
      }
    }
  },

  drawBuildings(s, ctx) {
    const L = this.layers, C = this.COLORS;
    for (let i = 0; i < ctx.buildingInfos.length; i++) {
      const info = ctx.buildingInfos[i];
      const bad = info.violations.length > 0;
      if (s.view.layers.sidewalks && s.sidewalks.building > 0) {
        this.poly(L.building, info.swPoly, { fill: C.sidewalk, stroke: '#b9b9b0', 'stroke-width': 0.05 });
      }
      const el = this.poly(L.building, info.poly, {
        fill: bad ? C.buildingBad : C.building,
        stroke: bad ? C.buildingBadStroke : C.buildingStroke,
        'stroke-width': 2, 'vector-effect': 'non-scaling-stroke',
        'data-building': i, cursor: info.b.locked ? 'not-allowed' : 'move'
      });
      if (i === s.selectedBuilding) {
        Util.attr(el, { stroke: '#2f6fed', 'stroke-width': 2.6 });
      }
      const b = info.b;
      const sizeTxt = `${Util.fmt(b.width)} × ${Util.fmt(b.depth)} m`;
      this.wtext(L.building, b.x, b.y - 1, b.name, Math.min(2.2, b.width / 8), {
        'font-weight': '700', fill: bad ? '#7c1d1d' : '#5c4a1e', 'pointer-events': 'none',
        transform: `rotate(${b.rotation} ${b.x} ${b.y})`
      });
      this.wtext(L.building, b.x, b.y + 1.6, sizeTxt, Math.min(1.6, b.width / 11), {
        fill: bad ? '#7c1d1d' : '#7a6534', 'pointer-events': 'none',
        transform: `rotate(${b.rotation} ${b.x} ${b.y})`
      });
      /* entrance marker on the principal building */
      if (i === 0 && ctx.entrancePt) {
        Util.svgEl('circle', { cx: ctx.entrancePt.x, cy: ctx.entrancePt.y, r: 0.6, fill: '#2f6fed' }, L.building);
        this.wtext(L.building, ctx.entrancePt.x, ctx.entrancePt.y - 1.4, 'ENTRANCE', 0.9, { fill: '#2f6fed', 'font-weight': '700' });
      }
    }
  },

  drawZones(s) {
    const L = this.layers, C = this.COLORS;
    for (const z of s.zones) {
      if (z.type === 'landscape' ? !s.view.layers.landscape : !s.view.layers.zones) continue;
      const pts = G.rectPoly(z.x + z.w / 2, z.y + z.h / 2, z.w, z.h, z.angle || 0);
      if (z.type === 'noparking') {
        this.poly(L.zones, pts, { fill: C.zoneNo, 'fill-opacity': 0.25, stroke: C.zoneNo, 'stroke-width': 0.08, 'data-zone': z.id });
        this.poly(L.zones, pts, { fill: 'url(#patNoPark)', 'fill-opacity': 0.5, stroke: 'none', 'pointer-events': 'none' });
        this.wtext(L.zones, z.x + z.w / 2, z.y + z.h / 2, 'NO PARKING', Math.min(1.2, z.w / 8), { fill: '#8f2323', 'font-weight': '700', 'pointer-events': 'none' });
      } else if (z.type === 'landscape') {
        this.poly(L.zones, pts, { fill: C.zoneLs, 'fill-opacity': 0.7, stroke: '#5a7f4a', 'stroke-width': 0.08, 'data-zone': z.id });
        this.wtext(L.zones, z.x + z.w / 2, z.y + z.h / 2, '\u{1F333}', Math.min(1.8, Math.min(z.w, z.h) * 0.6), { 'pointer-events': 'none' });
      } else if (z.type === 'crossing') {
        const g = Util.svgEl('g', { transform: `rotate(${z.angle || 0} ${z.x + z.w / 2} ${z.y + z.h / 2})` }, L.zones);
        Util.svgEl('rect', { x: z.x, y: z.y, width: z.w, height: z.h, fill: C.crossing, 'data-zone': z.id }, g);
        const stripes = Math.max(3, Math.floor(z.w / 0.8));
        for (let i = 0; i < stripes; i += 2) {
          Util.svgEl('rect', {
            x: z.x + i * z.w / stripes, y: z.y + 0.1, width: z.w / stripes, height: z.h - 0.2,
            fill: '#ffffff', 'pointer-events': 'none'
          }, g);
        }
      }
    }
  },

  drawAccessPoints(s, ctx) {
    const L = this.layers;
    for (const ai of ctx.apInfos) {
      const { ap, pos, tang, normal } = ai;
      const g = Util.svgEl('g', { 'data-ap': ap.id, cursor: 'pointer' }, L.access);
      const w = ap.width;
      /* opening across the boundary */
      this.line(g, G.add(pos, G.scale(tang, -w / 2)), G.add(pos, G.scale(tang, w / 2)),
        { stroke: '#ffffff', 'stroke-width': 0.5 });
      const mkTri = (offT, dirSign, color) => {
        const c = G.add(pos, G.scale(tang, offT));
        const tip = G.add(c, G.scale(normal, 2.2 * dirSign));
        const b1 = G.add(G.add(c, G.scale(normal, 0.2 * dirSign)), G.scale(tang, 0.9));
        const b2 = G.add(G.add(c, G.scale(normal, 0.2 * dirSign)), G.scale(tang, -0.9));
        this.poly(g, [tip, b1, b2], { fill: color, stroke: 'none' });
      };
      if (ap.type === 'in' || ap.type === 'both') mkTri(ap.type === 'both' ? -w / 4 : 0, 1, '#1e9e5a');
      if (ap.type === 'out' || ap.type === 'both') mkTri(ap.type === 'both' ? w / 4 : 0, -1, '#d64545');
      const lbl = ap.type === 'in' ? 'IN' : ap.type === 'out' ? 'OUT' : 'IN / OUT';
      const lp = G.add(pos, G.scale(normal, -1.6));
      this.wtext(g, lp.x, lp.y, `${lbl} ${Util.fmt(w)} m`, 1.0, { fill: '#2b2f36', 'font-weight': '700' });
    }
  },

  drawDimensions(s, ctx) {
    const L = this.layers;
    const bb = G.bbox(ctx.landPoly);
    if (s.land.mode === 'rect' && !s.land.rotation) {
      /* overall land dimensions outside the boundary */
      const roadN = s.roads.find(r => r.edge === 0), roadW = s.roads.find(r => r.edge === 3);
      const offTop = -(roadN ? roadN.width + 2 : 2.5);
      const offLeft = (roadW ? roadW.width + 2 : 2.5);
      this.dimension(L.dimensions, { x: bb.minX, y: bb.minY }, { x: bb.maxX, y: bb.minY }, offTop);
      this.dimension(L.dimensions, { x: bb.minX, y: bb.minY }, { x: bb.minX, y: bb.maxY }, offLeft);
      /* front setback dimension */
      const f = ctx.frontEdgeIdx;
      if (ctx.insets[f] > 0) {
        const e = ctx.edges[f];
        const n = G.inwardNormal(ctx.landPoly, f);
        const p0 = G.scale(G.add(e.a, e.b), 0.5);
        this.dimension(L.dimensions, p0, G.add(p0, G.scale(n, ctx.insets[f])), 2.5, { size: 0.9 });
      }
    } else {
      /* boundary segment lengths */
      for (const e of ctx.edges) {
        const n = G.inwardNormal(ctx.landPoly, e.idx);
        const mid = G.scale(G.add(e.a, e.b), 0.5);
        const p = G.add(mid, G.scale(n, -1.4));
        let ang = G.r2d(Math.atan2(e.b.y - e.a.y, e.b.x - e.a.x));
        if (ang > 90 || ang < -90) ang += 180;
        this.wtext(L.dimensions, p.x, p.y, Util.fmtLen(e.len), 1.1, {
          fill: this.COLORS.dim, transform: `rotate(${ang} ${p.x} ${p.y})`
        });
      }
    }
    /* user dimensions */
    for (const d of s.dimensions) {
      this.dimension(L.dimensions, d.a, d.b, 1.2, { color: '#0e7490' });
      const el = this.line(L.dimensions, d.a, d.b, { stroke: 'transparent', 'stroke-width': 1, 'data-dim': d.id });
      el.style.pointerEvents = 'stroke';
    }
  },

  drawRowLabels(layout) {
    const L = this.layers;
    /* count stalls per (band,row) and label at the row start */
    const groups = {};
    for (const st of layout.stalls) {
      if (st.band < 0 || st.type === 'accessAisle') continue;
      const k = st.band + '_' + st.row;
      (groups[k] = groups[k] || []).push(st);
    }
    for (const k in groups) {
      const arr = groups[k];
      let first = arr[0];
      for (const st of arr) if (st.slot < first.slot) first = st;
      const dir = (layout.bandAxis && layout.bandAxis[first.band]) || G.fromFrame({ x: 1, y: 0 }, layout.meta.orient);
      const p = { x: first.cx - dir.x * (layout.meta.geom.pitch / 2 + 1.2), y: first.cy - dir.y * (layout.meta.geom.pitch / 2 + 1.2) };
      this.wtext(L.labels, p.x, p.y, String(arr.length), 1.2, { fill: '#6b7280', 'font-weight': '700' });
    }
  },

  /* Selection outline + building handles */
  drawSelection(s, ctx, layout) {
    const sel = Interact.sel;
    const L = this.layers;
    if (!sel) return;
    if (sel.type === 'stall' && layout) {
      const st = layout.stalls.find(x => x.key === sel.id);
      if (st) this.poly(L.handles, st.poly, { fill: 'none', stroke: '#f59e0b', 'stroke-width': 2.4, 'vector-effect': 'non-scaling-stroke' });
    } else if (sel.type === 'seg' && layout) {
      const seg = layout.aisles.find(x => x.id === sel.id);
      if (seg) this.poly(L.handles, seg.poly, { fill: 'none', stroke: '#f59e0b', 'stroke-width': 2.4, 'vector-effect': 'non-scaling-stroke', 'stroke-dasharray': '8 5' });
    } else if (sel.type === 'zone') {
      const z = s.zones.find(x => x.id === sel.id);
      if (z) {
        const pts = G.rectPoly(z.x + z.w / 2, z.y + z.h / 2, z.w, z.h, z.angle || 0);
        this.poly(L.handles, pts, { fill: 'none', stroke: '#f59e0b', 'stroke-width': 2.4, 'vector-effect': 'non-scaling-stroke' });
      }
    } else if (sel.type === 'ap') {
      const ai = ctx.apInfos.find(x => x.ap.id === sel.id);
      if (ai) Util.svgEl('circle', {
        cx: ai.pos.x, cy: ai.pos.y, r: ai.ap.width / 2 + 0.6, fill: 'none',
        stroke: '#f59e0b', 'stroke-width': 2.4, 'vector-effect': 'non-scaling-stroke'
      }, L.handles);
    } else if (sel.type === 'building') {
      const info = ctx.buildingInfos[sel.idx];
      if (!info || info.b.locked) return;
      const px = 7 / this.view.k;   // handle half-size in world units
      for (let ci = 0; ci < 4; ci++) {
        const p = info.poly[ci];
        Util.svgEl('rect', {
          x: p.x - px, y: p.y - px, width: 2 * px, height: 2 * px,
          fill: '#ffffff', stroke: '#2f6fed', 'stroke-width': 1.6,
          'vector-effect': 'non-scaling-stroke', 'data-handle': 'resize', 'data-corner': ci,
          cursor: 'nwse-resize'
        }, L.handles);
      }
      /* rotation handle above the top edge midpoint */
      const b = info.b;
      const topMid = G.rotAround({ x: b.x, y: b.y - b.depth / 2 - 24 / this.view.k }, { x: b.x, y: b.y }, G.d2r(b.rotation));
      const topEdge = G.rotAround({ x: b.x, y: b.y - b.depth / 2 }, { x: b.x, y: b.y }, G.d2r(b.rotation));
      this.line(L.handles, topEdge, topMid, { stroke: '#2f6fed', 'stroke-width': 1.2, 'vector-effect': 'non-scaling-stroke' });
      Util.svgEl('circle', {
        cx: topMid.x, cy: topMid.y, r: px * 1.1, fill: '#ffffff', stroke: '#2f6fed',
        'stroke-width': 1.6, 'vector-effect': 'non-scaling-stroke',
        'data-handle': 'rotate', cursor: 'grab'
      }, L.handles);
    } else if (sel.type === 'vertex') {
      /* polygon vertex handles are drawn by drawPolyDraft */
    }
  },

  /** Draft polygon while drawing / editing custom land. */
  drawPolyDraft() {
    const L = this.layers;
    const s = App.state();
    const mode = Interact.polyMode;
    if (s.land.mode !== 'poly') return;
    const pts = mode === 'draw' ? Interact.polyDraft : s.land.polygon;
    if (!pts || !pts.length) return;
    if (pts.length > 1) {
      this.poly(L.temp, pts, {
        fill: mode === 'draw' ? 'rgba(47,111,237,0.08)' : 'none',
        stroke: '#2f6fed', 'stroke-width': 1.6,
        'vector-effect': 'non-scaling-stroke',
        'stroke-dasharray': mode === 'draw' ? '7 5' : 'none'
      });
    }
    for (let i = 0; i < pts.length; i++) {
      const a = pts[i], b = pts[(i + 1) % pts.length];
      if (i < pts.length - 1 || mode !== 'draw') {
        const mid = G.scale(G.add(a, b), 0.5);
        this.wtext(L.temp, mid.x, mid.y - 0.9, Util.fmtLen(G.dist(a, b)), 1.0, { fill: '#2f6fed' });
      }
    }
    const showHandles = mode === 'draw' || mode === 'edit';
    if (showHandles) {
      const r = 5 / this.view.k;
      pts.forEach((p, i) => {
        Util.svgEl('circle', {
          cx: p.x, cy: p.y, r, fill: '#fff', stroke: '#2f6fed', 'stroke-width': 1.6,
          'vector-effect': 'non-scaling-stroke', 'data-vertex': i, cursor: 'move'
        }, L.temp);
      });
    }
  },

  /* ── screen-space overlay: north arrow + scale bar ── */
  drawOverlay() {
    if (!this.overlay) return;
    this.overlay.textContent = '';
    const rect = this.svg.getBoundingClientRect();
    const s = App.state();

    /* North arrow (top-right) */
    const nx = rect.width - 44, ny = 46;
    const gN = Util.svgEl('g', { transform: `translate(${nx},${ny}) rotate(${s.land.northAngle || 0})` }, this.overlay);
    Util.svgEl('circle', { cx: 0, cy: 0, r: 22, fill: 'rgba(255,255,255,0.9)', stroke: '#64748b', 'stroke-width': 1 }, gN);
    Util.svgEl('path', { d: 'M0,-16 L6,8 L0,3 Z', fill: '#d64545' }, gN);
    Util.svgEl('path', { d: 'M0,-16 L-6,8 L0,3 Z', fill: '#334155' }, gN);
    const nT = Util.svgEl('text', {
      x: 0, y: -26, 'text-anchor': 'middle', 'font-size': 12, 'font-weight': 700,
      fill: '#334155', 'font-family': 'Arial, sans-serif'
    }, gN);
    nT.textContent = 'N';

    /* Scale bar (bottom-left) */
    const nice = [1, 2, 5, 10, 20, 50, 100, 200];
    let unit = nice[0];
    for (const u of nice) { if (u * this.view.k <= 140) unit = u; }
    const segPx = unit * this.view.k / 2;
    const bx = 16, by = rect.height - 26;
    const gS = Util.svgEl('g', { transform: `translate(${bx},${by})` }, this.overlay);
    Util.svgEl('rect', { x: -6, y: -18, width: segPx * 2 + 46, height: 30, fill: 'rgba(255,255,255,0.85)', rx: 4 }, gS);
    Util.svgEl('rect', { x: 0, y: -4, width: segPx, height: 6, fill: '#1f2937' }, gS);
    Util.svgEl('rect', { x: segPx, y: -4, width: segPx, height: 6, fill: '#ffffff', stroke: '#1f2937', 'stroke-width': 1 }, gS);
    const lbls = [[0, '0'], [segPx, Util.fmt(unit / 2)], [segPx * 2, Util.fmt(unit) + ' m']];
    for (const [x, txt] of lbls) {
      const t = Util.svgEl('text', {
        x, y: -8, 'text-anchor': 'middle', 'font-size': 10, fill: '#1f2937', 'font-family': 'Arial, sans-serif'
      }, gS);
      t.textContent = txt;
    }
  }
};

/* ═══════════════════════ 8. Interact — pointer tools ═══════════════════════ */

const Interact = {
  svg: null,
  tool: 'select',
  sel: null,                 // {type:'stall'|'seg'|'building'|'zone'|'ap'|'dim', id|idx}
  drag: null,                // active drag operation
  polyMode: null,            // null | 'draw' | 'edit'
  polyDraft: [],
  measure: null,             // {a, b}
  dimPending: null,          // first click of the dimension tool
  spaceDown: false,
  pointers: new Map(),       // for pinch zoom
  lastPinchDist: 0,

  init(svg) {
    this.svg = svg;
    svg.addEventListener('pointerdown', e => this.onDown(e));
    svg.addEventListener('pointermove', e => this.onMove(e));
    svg.addEventListener('pointerup', e => this.onUp(e));
    svg.addEventListener('pointercancel', e => this.onUp(e));
    svg.addEventListener('wheel', e => {
      e.preventDefault();
      const r = svg.getBoundingClientRect();
      Renderer.zoomAt(Math.pow(1.0015, -e.deltaY), e.clientX - r.left, e.clientY - r.top);
    }, { passive: false });
    svg.addEventListener('dblclick', e => this.onDblClick(e));
    window.addEventListener('keydown', e => this.onKey(e));
    window.addEventListener('keyup', e => { if (e.code === 'Space') { this.spaceDown = false; this.updateCursor(); } });
  },

  setTool(name) {
    this.tool = name;
    this.dimPending = null;
    document.querySelectorAll('.tool-btn[data-tool]').forEach(b => {
      b.classList.toggle('active', b.dataset.tool === name);
    });
    const hints = {
      select: 'Click to select. Drag the building to move it; handles resize / rotate.',
      pan: 'Drag to pan the plan.',
      measure: 'Click and drag to measure a distance.',
      dimension: 'Click two points to add a dimension annotation.',
      zoneNo: 'Drag a rectangle to add a NO-PARKING zone.',
      zoneLs: 'Drag a rectangle to add a landscape island.',
      crossing: 'Click on an aisle to place a pedestrian crossing.',
      addStall: 'Click to add a parking stall manually (aligned to the current layout).',
      addAccess: 'Click on an edge that has a road with access allowed.'
    };
    UI.hint(hints[name] || 'Ready.');
    this.updateCursor();
  },

  updateCursor() {
    const c = this.svg.classList;
    c.toggle('tool-pan', this.tool === 'pan' || this.spaceDown);
    c.toggle('tool-draw', ['measure', 'dimension', 'zoneNo', 'zoneLs', 'crossing', 'addStall', 'addAccess'].includes(this.tool) || this.polyMode === 'draw');
  },

  evtWorld(e) {
    const r = this.svg.getBoundingClientRect();
    return Renderer.s2w({ x: e.clientX - r.left, y: e.clientY - r.top });
  },

  /** Pointer capture, tolerant of already-released/synthetic pointers. */
  capture(e) {
    try { this.svg.setPointerCapture(e.pointerId); } catch (err) { /* pointer already gone */ }
  },

  /** Read a data-* attribute from the event target or its nearest ancestor
      (markers like access points set the attribute on a <g> wrapper). */
  dataAt(el, attr) {
    const n = el && el.closest ? el.closest('[' + attr + ']') : null;
    return n ? n.getAttribute(attr) : undefined;
  },

  /** Start a drag whose undo snapshot is only committed once the drag
      actually mutates state — plain clicks never pollute the undo stack. */
  beginDrag(drag) {
    drag.snap = JSON.stringify(State.s);
    drag.pushed = false;
    this.drag = drag;
  },
  markDragMutation() {
    const d = this.drag;
    if (!d || d.pushed || !d.snap) return;
    State.undoStack.push(d.snap);
    if (State.undoStack.length > State.UNDO_LIMIT) State.undoStack.shift();
    State.redoStack.length = 0;
    UI.updateUndoButtons();
    d.pushed = true;
  },

  snap(p, step) {
    const s = App.state();
    if (!s.view.snap) return { x: p.x, y: p.y };
    const st = step || 0.5;
    let out = { x: Math.round(p.x / st) * st, y: Math.round(p.y / st) * st };
    /* snap to land edges when close */
    const ctx = App.ctx;
    if (ctx && ctx.valid) {
      const tol = 8 / Renderer.view.k;
      for (const e of ctx.edges) {
        const cp = G.closestPointOnSeg(p, e.a, e.b);
        if (G.dist(p, cp) < tol) { out = cp; break; }
      }
    }
    return out;
  },

  /* ── pointer down ─────────────────────────────────────────────── */
  onDown(e) {
    this.svg.focus({ preventScroll: true });
    this.pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
    if (this.pointers.size === 2) {
      const [p1, p2] = [...this.pointers.values()];
      this.lastPinchDist = Math.hypot(p1.x - p2.x, p1.y - p2.y);
      this.drag = { kind: 'pinch' };
      return;
    }
    const w = this.evtWorld(e);
    const t = e.target;
    const panWanted = this.tool === 'pan' || this.spaceDown || e.button === 1;

    if (panWanted) {
      this.drag = { kind: 'pan', sx: e.clientX, sy: e.clientY, tx: Renderer.view.tx, ty: Renderer.view.ty };
      this.svg.classList.add('panning');
      this.capture(e);
      return;
    }
    if (e.button !== 0) return;
    this.capture(e);

    /* polygon drawing / editing takes precedence */
    if (this.polyMode === 'draw') {
      const p = this.snap(w);
      const last = this.polyDraft[this.polyDraft.length - 1];
      /* a click on (or next to) the previous point = the finish gesture —
         the native dblclick event cannot fire because each render swaps
         the elements under the cursor */
      if (last && G.dist(p, last) < 8 / Renderer.view.k) {
        UI.finishPolygon();
        return;
      }
      this.polyDraft.push(p);
      const vc = Util.el('polyVertexCount');
      if (vc) vc.textContent = String(this.polyDraft.length);
      UI.hint(`${this.polyDraft.length} point(s). Click the last point again, double-click, or press “Finish boundary” to close.`);
      App.renderOnly();
      return;
    }
    const vertexAttr = this.dataAt(t, 'data-vertex');
    if (this.polyMode === 'edit' && vertexAttr !== undefined) {
      this.beginDrag({ kind: 'vertex', idx: parseInt(vertexAttr, 10) });
      return;
    }

    switch (this.tool) {
      case 'measure':
        this.measure = { a: this.snap(w), b: this.snap(w) };
        this.drag = { kind: 'measure' };
        return;
      case 'dimension':
        if (!this.dimPending) {
          this.dimPending = this.snap(w);
          UI.hint('Dimension: click the second point.');
        } else {
          const s = App.state();
          State.pushUndo();
          s.dimensions.push({ id: Util.uid('dim'), a: this.dimPending, b: this.snap(w) });
          this.dimPending = null;
          App.renderOnly();
          UI.hint('Dimension added.');
        }
        return;
      case 'zoneNo':
      case 'zoneLs':
        this.drag = { kind: 'zoneDraw', type: this.tool === 'zoneNo' ? 'noparking' : 'landscape', a: this.snap(w) };
        return;
      case 'crossing': {
        const s = App.state();
        State.pushUndo();
        const ang = App.layout ? App.layout.meta.orient : 0;
        s.zones.push({ id: Util.uid('z'), type: 'crossing', x: w.x - 2, y: w.y - 1.25, w: 4, h: 2.5, angle: ang });
        App.recalc(false);
        UI.hint('Pedestrian crossing placed.');
        return;
      }
      case 'addStall': {
        const s = App.state();
        State.pushUndo();
        const axis = App.layout ? (App.layout.meta.geom.key === 'parallel' ? App.layout.meta.orient : App.layout.meta.geom.axisLow + App.layout.meta.orient) : 90;
        const before = App.layout ? App.layout.stalls.length : 0;
        s.manual.added.push({ id: Util.uid('ms'), x: this.snap(w).x, y: this.snap(w).y, axis, type: 'manual' });
        App.recalc(false);
        const after = App.layout ? App.layout.stalls.length : 0;
        if (after <= before) {
          s.manual.added.pop();
          App.recalc(false);
          UI.hint('Stall rejected: it would overlap an obstacle, setback or another stall.');
        } else {
          UI.hint('Manual stall added.');
        }
        return;
      }
      case 'addAccess':
        this.tryAddAccess(w);
        return;
    }

    /* ── select tool ── */
    const handle = this.dataAt(t, 'data-handle');
    if (handle === 'resize' || handle === 'rotate') {
      const s = App.state();
      const b = s.buildings[s.selectedBuilding];
      if (b && !b.locked) {
        this.beginDrag(handle === 'resize'
          ? { kind: 'resize', corner: parseInt(this.dataAt(t, 'data-corner'), 10), b }
          : { kind: 'rotate', b });
      }
      return;
    }
    const bldgAttr = this.dataAt(t, 'data-building');
    if (bldgAttr !== undefined) {
      const s = App.state();
      const idx = parseInt(bldgAttr, 10);
      s.selectedBuilding = idx;
      this.select({ type: 'building', idx });
      const b = s.buildings[idx];
      UI.syncBuildingFields();
      if (b && !b.locked) {
        this.beginDrag({ kind: 'moveBuilding', b, ox: b.x - w.x, oy: b.y - w.y });
      }
      return;
    }
    const stallAttr = this.dataAt(t, 'data-stall');
    if (stallAttr) { this.select({ type: 'stall', id: stallAttr }); return; }
    const segAttr = this.dataAt(t, 'data-seg');
    if (segAttr) { this.select({ type: 'seg', id: segAttr }); return; }
    const apAttr = this.dataAt(t, 'data-ap');
    if (apAttr) {
      this.select({ type: 'ap', id: apAttr });
      const s = App.state();
      const ap = s.accessPoints.find(a => a.id === apAttr);
      if (ap) this.beginDrag({ kind: 'moveAp', ap });
      return;
    }
    const zoneAttr = this.dataAt(t, 'data-zone');
    if (zoneAttr) {
      this.select({ type: 'zone', id: zoneAttr });
      const s = App.state();
      const z = s.zones.find(x => x.id === zoneAttr);
      if (z) this.beginDrag({ kind: 'moveZone', z, ox: z.x - w.x, oy: z.y - w.y });
      return;
    }
    const dimAttr = this.dataAt(t, 'data-dim');
    if (dimAttr) { this.select({ type: 'dim', id: dimAttr }); return; }
    this.select(null);
  },

  /* ── pointer move ─────────────────────────────────────────────── */
  onMove(e) {
    if (this.pointers.has(e.pointerId)) this.pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
    const w = this.evtWorld(e);
    Util.el('statCoords').textContent = `x: ${Util.fmt(w.x, 2)} m , y: ${Util.fmt(w.y, 2)} m`;

    if (this.drag && this.drag.kind === 'pinch' && this.pointers.size === 2) {
      const [p1, p2] = [...this.pointers.values()];
      const d = Math.hypot(p1.x - p2.x, p1.y - p2.y);
      if (this.lastPinchDist > 0) {
        const r = this.svg.getBoundingClientRect();
        const cx = (p1.x + p2.x) / 2 - r.left, cy = (p1.y + p2.y) / 2 - r.top;
        Renderer.zoomAt(d / this.lastPinchDist, cx, cy);
      }
      this.lastPinchDist = d;
      return;
    }
    if (!this.drag) return;
    const d = this.drag;
    switch (d.kind) {
      case 'pan':
        Renderer.view.tx = d.tx + (e.clientX - d.sx);
        Renderer.view.ty = d.ty + (e.clientY - d.sy);
        Renderer.applyView();
        Renderer.drawOverlay();
        break;
      case 'moveBuilding': {
        this.markDragMutation();
        const p = this.snap({ x: w.x + d.ox, y: w.y + d.oy });
        d.b.x = Math.round(p.x * 100) / 100;
        d.b.y = Math.round(p.y * 100) / 100;
        UI.syncBuildingFields();
        App.schedulePreview();
        break;
      }
      case 'resize': {
        this.markDragMutation();
        const b = d.b;
        const rot = G.d2r(b.rotation);
        /* pointer in building-local frame (origin = current centre) */
        const local = G.rot(G.sub(w, { x: b.x, y: b.y }), -rot);
        const oppSign = [{ x: 1, y: 1 }, { x: -1, y: 1 }, { x: -1, y: -1 }, { x: 1, y: -1 }][d.corner];
        const opp = { x: oppSign.x * b.width / 2, y: oppSign.y * b.depth / 2 };
        let newW = Math.abs(local.x - opp.x), newD = Math.abs(local.y - opp.y);
        const s = App.state();
        if (s.view.snap) { newW = Math.round(newW * 2) / 2; newD = Math.round(newD * 2) / 2; }
        newW = Util.clamp(newW, 3, 500); newD = Util.clamp(newD, 3, 500);
        const cLocal = { x: opp.x + (local.x >= opp.x ? newW : -newW) / 2, y: opp.y + (local.y >= opp.y ? newD : -newD) / 2 };
        const cWorld = G.add({ x: b.x, y: b.y }, G.rot(cLocal, rot));
        b.width = newW; b.depth = newD; b.x = cWorld.x; b.y = cWorld.y;
        b.area = Math.round(newW * newD * 100) / 100;
        UI.syncBuildingFields();
        App.schedulePreview();
        break;
      }
      case 'rotate': {
        this.markDragMutation();
        const b = d.b;
        let ang = G.r2d(Math.atan2(w.y - b.y, w.x - b.x)) + 90;
        const s = App.state();
        if (s.view.snap) ang = Math.round(ang / 5) * 5;
        b.rotation = ((Math.round(ang * 10) / 10 + 180) % 360) - 180;
        UI.syncBuildingFields();
        App.schedulePreview();
        break;
      }
      case 'moveAp': {
        const ctx = App.ctx;
        const ap = d.ap;
        if (ctx && ctx.valid && ap.edge >= 0 && ap.edge < ctx.edges.length) {
          this.markDragMutation();
          const e2 = ctx.edges[ap.edge];
          const cp = G.closestPointOnSeg(w, e2.a, e2.b);
          const tRaw = G.dist(e2.a, cp) / e2.len;
          const halfW = (ap.width / 2) / e2.len;
          ap.t = Util.clamp(tRaw, halfW, 1 - halfW);
          App.schedulePreview();
        }
        break;
      }
      case 'moveZone': {
        this.markDragMutation();
        const p = this.snap({ x: w.x + d.ox, y: w.y + d.oy });
        d.z.x = p.x; d.z.y = p.y;
        App.schedulePreview();
        break;
      }
      case 'vertex': {
        const s = App.state();
        const p = this.snap(w);
        if (s.land.polygon[d.idx]) {
          this.markDragMutation();
          s.land.polygon[d.idx] = { x: p.x, y: p.y };
          App.schedulePreview();
        }
        break;
      }
      case 'zoneDraw':
        d.b = this.snap(w);
        this.drawTemp();
        break;
      case 'measure':
        this.measure.b = this.snap(w);
        this.drawTemp();
        break;
    }
  },

  /* ── pointer up ───────────────────────────────────────────────── */
  onUp(e) {
    this.pointers.delete(e.pointerId);
    this.svg.classList.remove('panning');
    const d = this.drag;
    this.drag = null;
    this.lastPinchDist = 0;
    if (!d) return;
    switch (d.kind) {
      case 'moveBuilding':
      case 'resize':
      case 'rotate':
      case 'moveAp':
      case 'moveZone':
      case 'vertex':
        /* run the full optimizer only when the drag actually changed state */
        if (d.pushed) App.recalc(true);
        break;
      case 'zoneDraw': {
        if (d.b) {
          const x0 = Math.min(d.a.x, d.b.x), y0 = Math.min(d.a.y, d.b.y);
          const zw = Math.abs(d.b.x - d.a.x), zh = Math.abs(d.b.y - d.a.y);
          if (zw >= 1 && zh >= 1) {
            State.pushUndo();
            App.state().zones.push({ id: Util.uid('z'), type: d.type, x: x0, y: y0, w: zw, h: zh, angle: 0 });
            App.recalc(true);
            UI.hint(d.type === 'noparking' ? 'No-parking zone added.' : 'Landscape island added.');
          } else {
            App.renderOnly();
          }
        }
        break;
      }
      case 'measure':
        UI.hint('Distance: ' + Util.fmtLen(G.dist(this.measure.a, this.measure.b)));
        break;
    }
  },

  onDblClick(e) {
    if (this.polyMode === 'draw') {
      e.preventDefault();
      UI.finishPolygon();
    }
  },

  onKey(e) {
    const tag = document.activeElement && document.activeElement.tagName;
    const typing = tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT';
    if (e.code === 'Space' && !typing) { this.spaceDown = true; this.updateCursor(); e.preventDefault(); return; }
    /* while typing, leave Ctrl+Z / Ctrl+Y to the browser's native text
       undo — app-level undo stays available via the toolbar buttons */
    if (typing) return;
    if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'z') { e.preventDefault(); UI.doUndo(); return; }
    if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'y') { e.preventDefault(); UI.doRedo(); return; }
    switch (e.key) {
      case 'Delete': case 'Backspace': this.deleteSelection(); break;
      case 'Escape':
        if (this.polyMode === 'draw') { this.polyDraft = []; this.polyMode = null; App.renderOnly(); }
        this.dimPending = null; this.measure = null;
        this.setTool('select'); this.select(null);
        break;
      case 'v': case 'V': this.setTool('select'); break;
      case 'h': case 'H': this.setTool('pan'); break;
      case 'm': case 'M': this.setTool('measure'); break;
      case 'd': case 'D': this.setTool('dimension'); break;
      case 'f': case 'F': Renderer.zoomFit(App.ctx); break;
      case 'g': case 'G': UI.toggleGrid(); break;
      case 's': case 'S': UI.toggleSnap(); break;
      case '+': case '=': Renderer.zoomAt(1.25, this.svg.clientWidth / 2, this.svg.clientHeight / 2); break;
      case '-': case '_': Renderer.zoomAt(0.8, this.svg.clientWidth / 2, this.svg.clientHeight / 2); break;
    }
  },

  /* ── selection ────────────────────────────────────────────────── */
  select(sel) {
    this.sel = sel;
    this.updateSelInfo();
    App.renderOnly();
  },

  deleteSelection() {
    const sel = this.sel;
    if (!sel) return;
    const s = App.state();
    if (sel.type === 'stall') {
      State.pushUndo();
      const st = App.layout && App.layout.stalls.find(x => x.key === sel.id);
      if (st && st.manual) {
        s.manual.added = s.manual.added.filter(a => a.id !== sel.id);
      } else {
        s.manual.removed.push(sel.id);
      }
      this.sel = null;
      App.recalc(false);
    } else if (sel.type === 'seg') {
      this.deleteRow(sel.id);
    } else if (sel.type === 'zone') {
      State.pushUndo();
      s.zones = s.zones.filter(z => z.id !== sel.id);
      this.sel = null;
      App.recalc(true);
    } else if (sel.type === 'ap') {
      State.pushUndo();
      s.accessPoints = s.accessPoints.filter(a => a.id !== sel.id);
      this.sel = null;
      App.recalc(true);
    } else if (sel.type === 'dim') {
      State.pushUndo();
      s.dimensions = s.dimensions.filter(x => x.id !== sel.id);
      this.sel = null;
      App.renderOnly();
    } else if (sel.type === 'building') {
      UI.deleteBuilding();
    }
    this.updateSelInfo();
  },

  deleteRow(segId) {
    const s = App.state();
    const seg = App.layout && App.layout.aisles.find(x => x.id === segId);
    if (!seg) return;
    State.pushUndo();
    for (const st of App.layout.stalls) {
      if (st.segId === segId && !st.manual) s.manual.removed.push(st.key);
    }
    this.sel = null;
    App.recalc(false);
    UI.hint('Parking row removed. Use Regenerate to restore it.');
  },

  convertStall(type) {
    const sel = this.sel;
    if (!sel || sel.type !== 'stall') return;
    const s = App.state();
    State.pushUndo();
    if (type === 'regular') delete s.manual.typeOverrides[sel.id];
    else s.manual.typeOverrides[sel.id] = type;
    App.recalc(false);
    this.updateSelInfo();
  },

  shiftRow(segId, delta) {
    const seg = App.layout && App.layout.aisles.find(x => x.id === segId);
    if (!seg) return;
    const s = App.state();
    State.pushUndo();
    const k = String(seg.band);
    s.manual.rowShift[k] = (s.manual.rowShift[k] || 0) + delta;
    App.recalc(false);
  },

  reverseRow(segId) {
    const seg = App.layout && App.layout.aisles.find(x => x.id === segId);
    if (!seg) return;
    const s = App.state();
    State.pushUndo();
    const k = String(seg.band);
    s.manual.rowDir[k] = !s.manual.rowDir[k];
    App.recalc(false);
  },

  tryAddAccess(w) {
    const s = App.state();
    const ctx = App.ctx;
    if (!ctx || !ctx.valid) return;
    let best = null;
    for (const e of ctx.edges) {
      const cp = G.closestPointOnSeg(w, e.a, e.b);
      const dist = G.dist(w, cp);
      if (dist < 20 / Renderer.view.k + 2 && (!best || dist < best.dist)) best = { e, cp, dist };
    }
    if (!best) { UI.hint('Click closer to a land edge.'); return; }
    const road = s.roads.find(r => r.edge === best.e.idx);
    if (!road) { UI.hint(`The ${best.e.name} has no road — access is not possible there.`); return; }
    if (!road.accessAllowed) { UI.hint(`Vehicle access is not allowed on “${road.name}”.`); return; }
    State.pushUndo();
    const t = Util.clamp(G.dist(best.e.a, best.cp) / best.e.len, 0.05, 0.95);
    s.accessPoints.push({ id: Util.uid('ap'), edge: best.e.idx, t, width: 6, type: 'both' });
    this.setTool('select');
    App.recalc(true);
    UI.hint('Access point added.');
  },

  /* ── temp overlays (measure line, zone rubber-band) ───────────── */
  drawTemp() {
    App.renderOnly();
  },

  /** Called by App after every render — draws transient overlays. */
  renderTempOverlays() {
    const L = Renderer.layers.temp;
    if (this.measure) {
      const { a, b } = this.measure;
      Renderer.line(L, a, b, { stroke: '#0e7490', 'stroke-width': 2, 'vector-effect': 'non-scaling-stroke', 'stroke-dasharray': '6 4' });
      const mid = G.scale(G.add(a, b), 0.5);
      Renderer.wtext(L, mid.x, mid.y - 1, Util.fmtLen(G.dist(a, b)), 1.2, { fill: '#0e7490', 'font-weight': '700' });
    }
    if (this.dimPending) {
      Util.svgEl('circle', { cx: this.dimPending.x, cy: this.dimPending.y, r: 4 / Renderer.view.k, fill: '#0e7490' }, L);
    }
    if (this.drag && this.drag.kind === 'zoneDraw' && this.drag.b) {
      const { a, b } = this.drag;
      const col = this.drag.type === 'noparking' ? '#d64545' : '#3f7f3f';
      Util.svgEl('rect', {
        x: Math.min(a.x, b.x), y: Math.min(a.y, b.y),
        width: Math.abs(b.x - a.x), height: Math.abs(b.y - a.y),
        fill: col, 'fill-opacity': 0.15, stroke: col, 'stroke-width': 1.5,
        'vector-effect': 'non-scaling-stroke', 'stroke-dasharray': '6 4'
      }, L);
    }
  },

  /* ── floating selection card ──────────────────────────────────── */
  updateSelInfo() {
    const box = Util.el('selInfo');
    const sel = this.sel;
    if (!sel) { box.classList.add('is-hidden'); return; }
    const s = App.state();
    let html = '', actions = [];
    if (sel.type === 'stall') {
      const st = App.layout && App.layout.stalls.find(x => x.key === sel.id);
      if (!st) { box.classList.add('is-hidden'); return; }
      const typeName = { regular: 'Regular stall', accessible: 'Accessible stall', ev: 'EV stall', accessAisle: 'Access aisle', manual: 'Manual stall' }[st.type] || st.type;
      /* report the dimensions actually drawn (from the stall polygon) */
      const e01 = G.dist(st.poly[0], st.poly[1]), e12 = G.dist(st.poly[1], st.poly[2]);
      html = `<div class="si-title">${st.num != null ? 'P' + String(st.num).padStart(3, '0') + ' — ' : ''}${typeName}</div>
        ${Util.fmt(Math.min(e01, e12))} × ${Util.fmt(Math.max(e01, e12))} m — centre ${Util.fmt(st.cx, 1)}, ${Util.fmt(st.cy, 1)}`;
      actions = [
        ['To Accessible', () => this.convertStall('accessible')],
        ['To EV', () => this.convertStall('ev')],
        ['To Regular', () => this.convertStall('regular')],
        ['Delete', () => this.deleteSelection()]
      ];
    } else if (sel.type === 'seg') {
      const seg = App.layout && App.layout.aisles.find(x => x.id === sel.id);
      if (!seg) { box.classList.add('is-hidden'); return; }
      const count = App.layout.stalls.filter(x => x.segId === sel.id && x.type !== 'accessAisle').length;
      html = `<div class="si-title">Parking row / aisle</div>
        ${count} stall(s) · ${seg.oneWay ? 'one-way' : 'two-way'} aisle · ${seg.connected ? 'connected' : '<b style="color:#fca5a5">disconnected</b>'}`;
      actions = [
        ['◀ Shift 0.5 m', () => this.shiftRow(sel.id, -0.5)],
        ['Shift 0.5 m ▶', () => this.shiftRow(sel.id, 0.5)],
        ['Reverse direction', () => this.reverseRow(sel.id)],
        ['Delete row', () => this.deleteRow(sel.id)]
      ];
    } else if (sel.type === 'zone') {
      const z = s.zones.find(x => x.id === sel.id);
      if (!z) { box.classList.add('is-hidden'); return; }
      const names = { noparking: 'No-parking zone', landscape: 'Landscape island', crossing: 'Pedestrian crossing' };
      html = `<div class="si-title">${names[z.type]}</div>${Util.fmt(z.w, 1)} × ${Util.fmt(z.h, 1)} m — drag to move`;
      actions = [['Delete', () => this.deleteSelection()]];
    } else if (sel.type === 'ap') {
      const ap = s.accessPoints.find(x => x.id === sel.id);
      if (!ap) { box.classList.add('is-hidden'); return; }
      html = `<div class="si-title">Access point</div>Type: ${ap.type} · width ${Util.fmt(ap.width)} m — drag along the edge to move`;
      actions = [
        ['Cycle type', () => {
          State.pushUndo();
          ap.type = ap.type === 'both' ? 'in' : ap.type === 'in' ? 'out' : 'both';
          App.recalc(true); this.updateSelInfo();
        }],
        ['Width −0.5', () => { State.pushUndo(); ap.width = Math.max(3, ap.width - 0.5); App.recalc(true); this.updateSelInfo(); }],
        ['Width +0.5', () => { State.pushUndo(); ap.width = Math.min(15, ap.width + 0.5); App.recalc(true); this.updateSelInfo(); }],
        ['Delete', () => this.deleteSelection()]
      ];
    } else if (sel.type === 'building') {
      const b = s.buildings[sel.idx];
      if (!b) { box.classList.add('is-hidden'); return; }
      html = `<div class="si-title">${Util.escapeHTML(b.name)}</div>${Util.fmt(b.width)} × ${Util.fmt(b.depth)} m · ${Util.fmt(b.width * b.depth, 1)} m² · rot ${Util.fmt(b.rotation, 1)}°`;
      actions = [['Deselect', () => this.select(null)]];
    } else if (sel.type === 'dim') {
      html = `<div class="si-title">Dimension</div>`;
      actions = [['Delete', () => this.deleteSelection()]];
    }
    box.innerHTML = html + '<div class="si-actions"></div>';
    const wrap = box.querySelector('.si-actions');
    for (const [label, fn] of actions) {
      const btn = document.createElement('button');
      btn.textContent = label;
      btn.addEventListener('click', fn);
      wrap.appendChild(btn);
    }
    box.classList.remove('is-hidden');
  }
};

/* ═══════════════════════ 9. UI ═══════════════════════ */

const UI = {
  _focusSnapshot: null,

  init() {
    this.bindInputs();
    this.bindButtons();
    this.bindToolbar();
    this.bindDynamicLists();
    this.refreshPresetSelect();
    this.syncFromState();
  },

  hint(msg) { const el = Util.el('statHint'); if (el) el.textContent = msg; },

  /* ── generic data binding ─────────────────────────────────────── */
  bindInputs() {
    document.querySelectorAll('[data-bind]').forEach(inp => {
      const path = inp.dataset.bind;
      const type = inp.dataset.type || 'str';
      const discrete = inp.type === 'checkbox' || inp.type === 'radio' || inp.tagName === 'SELECT';

      if (!discrete) {
        inp.addEventListener('focus', () => { this._focusSnapshot = JSON.stringify(State.s); });
        inp.addEventListener('change', () => {
          if (this._focusSnapshot && this._focusSnapshot !== JSON.stringify(State.s)) {
            State.undoStack.push(this._focusSnapshot);
            if (State.undoStack.length > State.UNDO_LIMIT) State.undoStack.shift();
            State.redoStack.length = 0;
            this.updateUndoButtons();
          }
          this._focusSnapshot = null;
        });
      }

      inp.addEventListener(discrete ? 'change' : 'input', () => {
        let v;
        if (type === 'bool') v = inp.checked;
        else if (type === 'radio') { if (!inp.checked) return; v = inp.value; }
        else if (type === 'num' || type === 'numopt') {
          if (inp.value === '' && type === 'numopt') { v = null; }
          else {
            v = parseFloat(inp.value);
            const min = parseFloat(inp.min), max = parseFloat(inp.max);
            if (isNaN(v) || (!isNaN(min) && v < min) || (!isNaN(max) && v > max)) {
              inp.classList.add('invalid');
              this.hint(`Invalid value${isNaN(v) ? '' : ` — allowed range ${inp.min || '−∞'} to ${inp.max || '∞'}`}.`);
              return;
            }
          }
        } else v = inp.value;
        inp.classList.remove('invalid');
        if (discrete) State.pushUndo();
        State.set(path, v);
        this.onStateChanged(path);
      });
    });
  },

  /** Side effects when a bound value changes. */
  onStateChanged(path) {
    if (path === 'land.mode') this.syncModeVisibility();
    if (path === 'demand.method') this.syncModeVisibility();
    if (path.startsWith('optimization.weights')) this.syncWeightLabels();
    Util.el('topProjectName').textContent = State.s.project.name || 'Untitled Project';
    App.scheduleRecalc();
  },

  /** Push all state values into the bound inputs + rebuild dynamic UI. */
  syncFromState() {
    const s = State.s;
    document.querySelectorAll('[data-bind]').forEach(inp => {
      if (inp === document.activeElement) return;   // never clobber live typing
      const v = State.get(inp.dataset.bind);
      const type = inp.dataset.type || 'str';
      if (type === 'bool') inp.checked = !!v;
      else if (type === 'radio') inp.checked = (inp.value === String(v));
      else inp.value = v == null ? '' : v;
      inp.classList.remove('invalid');
    });
    document.querySelectorAll('#layerToggles [data-layer]').forEach(cb => {
      cb.checked = !!s.view.layers[cb.dataset.layer];
    });
    Util.el('btnGrid').classList.toggle('active', s.view.grid);
    Util.el('btnSnap').classList.toggle('active', s.view.snap);
    Util.el('topProjectName').textContent = s.project.name || 'Untitled Project';
    this.syncModeVisibility();
    this.syncWeightLabels();
    this.buildRoadsList();
    this.buildAccessList();
    this.buildMixedList();
    this.syncBuildingFields();
    this.updateUndoButtons();
  },

  /** Rebuild the access-point panel unless the user is editing inside it
      (called from App.recalc so canvas-side changes never leave it stale). */
  refreshAccessList() {
    const wrap = Util.el('accessList');
    if (wrap && wrap.contains(document.activeElement)) return;
    this.buildAccessList();
  },

  syncModeVisibility() {
    const s = State.s;
    Util.el('landRectFields').classList.toggle('is-hidden', s.land.mode !== 'rect');
    Util.el('landPolyFields').classList.toggle('is-hidden', s.land.mode !== 'poly');
    const m = s.demand.method;
    Util.el('demandPerArea').classList.toggle('is-hidden', m !== 'perArea');
    Util.el('demandPer100').classList.toggle('is-hidden', m !== 'per100');
    Util.el('demandFixed').classList.toggle('is-hidden', m !== 'fixed');
    Util.el('demandMixed').classList.toggle('is-hidden', m !== 'mixed');
  },

  syncWeightLabels() {
    const w = State.s.optimization.weights;
    const map = { wCountVal: w.count, wComplianceVal: w.compliance, wCirculationVal: w.circulation, wAccessVal: w.accessProx, wLandscapeVal: w.landscape, wSimpleVal: w.simplicity };
    for (const id in map) { const el = Util.el(id); if (el) el.textContent = ' ' + map[id]; }
  },

  updateUndoButtons() {
    Util.el('btnUndo').disabled = State.undoStack.length === 0;
    Util.el('btnRedo').disabled = State.redoStack.length === 0;
  },

  doUndo() {
    this._focusSnapshot = null;   // a stale focus snapshot must never survive a state swap
    if (State.undo()) { this.syncFromState(); App.recalc(true); this.hint('Undo.'); }
    this.updateUndoButtons();
  },
  doRedo() {
    this._focusSnapshot = null;
    if (State.redo()) { this.syncFromState(); App.recalc(true); this.hint('Redo.'); }
    this.updateUndoButtons();
  },

  toggleGrid() {
    State.s.view.grid = !State.s.view.grid;
    Util.el('btnGrid').classList.toggle('active', State.s.view.grid);
    App.renderOnly();
  },
  toggleSnap() {
    State.s.view.snap = !State.s.view.snap;
    Util.el('btnSnap').classList.toggle('active', State.s.view.snap);
    this.hint('Snapping ' + (State.s.view.snap ? 'on' : 'off') + '.');
  },

  /* ── top bar + toolbar ────────────────────────────────────────── */
  bindToolbar() {
    document.querySelectorAll('.tool-btn[data-tool]').forEach(b => {
      b.addEventListener('click', () => Interact.setTool(b.dataset.tool));
    });
    Util.el('btnAddAccess').addEventListener('click', () => Interact.setTool('addAccess'));
    Util.el('btnZoomIn').addEventListener('click', () => Renderer.zoomAt(1.25, Renderer.svg.clientWidth / 2, Renderer.svg.clientHeight / 2));
    Util.el('btnZoomOut').addEventListener('click', () => Renderer.zoomAt(0.8, Renderer.svg.clientWidth / 2, Renderer.svg.clientHeight / 2));
    Util.el('btnZoomFit').addEventListener('click', () => { Util.el('selScale').value = 'fit'; Renderer.zoomFit(App.ctx); });
    Util.el('selScale').addEventListener('change', e => {
      if (e.target.value === 'fit') Renderer.zoomFit(App.ctx);
      else Renderer.setScaleRatio(parseFloat(e.target.value), App.ctx);
    });
    Util.el('btnGrid').addEventListener('click', () => this.toggleGrid());
    Util.el('btnSnap').addEventListener('click', () => this.toggleSnap());
    Util.el('btnUndo').addEventListener('click', () => this.doUndo());
    Util.el('btnRedo').addEventListener('click', () => this.doRedo());
    Util.el('btnRegenerate').addEventListener('click', () => {
      State.pushUndo();
      State.s.manual = { removed: [], typeOverrides: {}, added: [], rowShift: {}, rowDir: {} };
      App.recalc(true);
      this.hint('Layout regenerated — manual edits cleared.');
    });
    Util.el('btnResetLayout').addEventListener('click', () => {
      State.pushUndo();
      State.s.manual = { removed: [], typeOverrides: {}, added: [], rowShift: {}, rowDir: {} };
      State.s.zones = [];
      State.s.dimensions = [];
      Interact.sel = null;
      Interact.updateSelInfo();
      App.recalc(true);
      this.hint('Layout reset — manual edits, zones and dimensions cleared.');
    });
    Util.el('btnPanelLeft').addEventListener('click', () => {
      Util.el('leftPanel').classList.toggle('open');
      Util.el('rightPanel').classList.remove('open');
    });
    Util.el('btnPanelRight').addEventListener('click', () => {
      Util.el('rightPanel').classList.toggle('open');
      Util.el('leftPanel').classList.remove('open');
    });
    document.querySelectorAll('#layerToggles [data-layer]').forEach(cb => {
      cb.addEventListener('change', () => {
        State.s.view.layers[cb.dataset.layer] = cb.checked;
        App.renderOnly();
      });
    });
  },

  /* ── buttons: land polygon, building, presets, exports ────────── */
  bindButtons() {
    /* Land polygon */
    Util.el('btnPolyStart').addEventListener('click', () => {
      State.pushUndo();
      State.s.land.mode = 'poly';
      Interact.polyMode = 'draw';
      Interact.polyDraft = [];
      this.syncFromState();
      this.hint('Click on the canvas to place boundary points; double-click or press “Finish boundary” to close.');
      App.recalc(true);
    });
    Util.el('btnPolyFinish').addEventListener('click', () => this.finishPolygon());
    Util.el('btnPolyUndo').addEventListener('click', () => {
      if (Interact.polyMode === 'draw' && Interact.polyDraft.length) {
        Interact.polyDraft.pop();
        Util.el('polyVertexCount').textContent = String(Interact.polyDraft.length);
      } else if (State.s.land.polygon.length) {
        State.pushUndo();
        State.s.land.polygon.pop();
        App.recalc(true);
      }
      App.renderOnly();
    });
    Util.el('btnPolyClear').addEventListener('click', () => {
      State.pushUndo();
      Interact.polyDraft = [];
      Interact.polyMode = 'draw';
      State.s.land.polygon = [];
      App.recalc(true);
      this.hint('Boundary cleared — draw a new one.');
    });
    Util.el('btnPolyEdit').addEventListener('click', () => {
      Interact.polyMode = Interact.polyMode === 'edit' ? null : 'edit';
      this.hint(Interact.polyMode === 'edit' ? 'Drag the vertex handles to edit the boundary.' : 'Vertex editing off.');
      App.renderOnly();
    });

    /* Building method radios */
    Util.el('bldgMethodWh').addEventListener('change', () => this.setBuildingMethod('wh'));
    Util.el('bldgMethodArea').addEventListener('change', () => this.setBuildingMethod('area'));

    /* Building numeric fields */
    const bindB = (id, fn) => {
      const inp = Util.el(id);
      inp.addEventListener('focus', () => { this._focusSnapshot = JSON.stringify(State.s); });
      inp.addEventListener('change', () => {
        if (this._focusSnapshot && this._focusSnapshot !== JSON.stringify(State.s)) {
          State.undoStack.push(this._focusSnapshot);
          State.redoStack.length = 0;
          this.updateUndoButtons();
        }
        this._focusSnapshot = null;
      });
      inp.addEventListener('input', () => {
        const b = this.selectedBuilding();
        if (!b) return;
        const v = parseFloat(inp.value);
        if (isNaN(v)) { inp.classList.add('invalid'); return; }
        inp.classList.remove('invalid');
        fn(b, v);
        App.scheduleRecalc();
      });
    };
    bindB('bldgWidth', (b, v) => { if (v >= 3 && !b.locked) { b.width = v; b.area = Math.round(b.width * b.depth * 100) / 100; } });
    bindB('bldgDepth', (b, v) => { if (v >= 3 && !b.locked) { b.depth = v; b.area = Math.round(b.width * b.depth * 100) / 100; } });
    bindB('bldgArea', (b, v) => { if (v >= 9 && !b.locked) { b.area = v; this.applyAreaMethod(b); } });
    bindB('bldgAspect', (b, v) => { if (v >= 0.2 && v <= 5 && !b.locked) { b.aspect = v; if (b.defMethod === 'area') this.applyAreaMethod(b); } });
    bindB('bldgX', (b, v) => { if (!b.locked) b.x = v; });
    bindB('bldgY', (b, v) => { if (!b.locked) b.y = v; });
    bindB('bldgRot', (b, v) => { if (!b.locked) b.rotation = Util.clamp(v, -180, 180); });
    Util.el('bldgLock').addEventListener('change', e => {
      const b = this.selectedBuilding();
      if (b) { State.pushUndo(); b.locked = e.target.checked; App.renderOnly(); }
    });

    Util.el('btnBldgCenter').addEventListener('click', () => {
      const b = this.selectedBuilding();
      if (!b || b.locked || !App.ctx.valid) return;
      State.pushUndo();
      const c = G.centroid(App.ctx.landPoly);
      b.x = Math.round(c.x * 100) / 100; b.y = Math.round(c.y * 100) / 100;
      this.syncBuildingFields();
      App.recalc(true);
    });
    Util.el('btnBldgAlign').addEventListener('click', () => this.alignBuilding(parseInt(Util.el('selAlignEdge').value, 10)));
    Util.el('btnBldgDuplicate').addEventListener('click', () => {
      const b = this.selectedBuilding();
      if (!b) return;
      State.pushUndo();
      const copy = Util.deepClone(b);
      copy.id = Util.uid('b');
      copy.name = 'Building ' + (State.s.buildings.length + 1);
      copy.x += b.width + 3;
      copy.locked = false;
      State.s.buildings.push(copy);
      State.s.selectedBuilding = State.s.buildings.length - 1;
      this.syncBuildingFields();
      App.recalc(true);
    });
    Util.el('btnBldgDelete').addEventListener('click', () => this.deleteBuilding());

    /* Demand: mixed rows */
    Util.el('btnAddMixedRow').addEventListener('click', () => {
      State.pushUndo();
      State.s.demand.mixed.push({ use: 'Office', area: 100, ratio: 40 });
      this.buildMixedList();
      App.scheduleRecalc();
    });

    /* Roads */
    Util.el('btnAddRoad').addEventListener('click', () => {
      State.pushUndo();
      const s = State.s;
      const used = new Set(s.roads.map(r => r.edge));
      const edgeCount = App.ctx.valid ? App.ctx.edges.length : 4;
      let edge = 0;
      for (let i = 0; i < edgeCount; i++) if (!used.has(i)) { edge = i; break; }
      s.roads.push({
        id: Util.uid('road'), edge, name: 'Road ' + (s.roads.length + 1), width: 15,
        classification: 'local', minWidth: 12, designation: s.roads.length ? 'side' : 'front',
        accessAllowed: true, maxEntrances: 1
      });
      this.buildRoadsList();
      App.scheduleRecalc();
    });

    /* Presets */
    Util.el('btnPresetSave').addEventListener('click', () => {
      const name = prompt('Preset name:', 'My municipality preset');
      if (!name) return;
      const store = State.loadPresetStore();
      store[name] = State.capturePreset();
      State.savePresetStore(store);
      this.refreshPresetSelect(name);
      this.hint(`Preset “${name}” saved (browser storage).`);
    });
    Util.el('btnPresetLoad').addEventListener('click', () => {
      const name = Util.el('selPreset').value;
      const store = State.loadPresetStore();
      if (!store[name]) return;
      State.pushUndo();
      State.applyPreset(store[name]);
      this.syncFromState();
      App.recalc(true);
      this.hint(`Preset “${name}” loaded.`);
    });
    Util.el('btnPresetDup').addEventListener('click', () => {
      const name = Util.el('selPreset').value;
      const store = State.loadPresetStore();
      if (!store[name]) return;
      const copy = name + ' (copy)';
      store[copy] = Util.deepClone(store[name]);
      State.savePresetStore(store);
      this.refreshPresetSelect(copy);
      this.hint(`Preset duplicated as “${copy}”.`);
    });
    Util.el('btnPresetDelete').addEventListener('click', () => {
      const name = Util.el('selPreset').value;
      const store = State.loadPresetStore();
      if (!(name in store)) return;
      if (!confirm(`Delete preset “${name}”?`)) return;
      delete store[name];
      State.savePresetStore(store);
      this.refreshPresetSelect();
      this.hint('Preset deleted.');
    });
    Util.el('btnPresetReset').addEventListener('click', () => {
      State.pushUndo();
      const builtin = State.builtinPresets()['Saudi Municipality — Custom Project Preset'];
      State.applyPreset(builtin);
      this.syncFromState();
      App.recalc(true);
      this.hint('Regulation fields reset to the built-in preset defaults.');
    });
    Util.el('btnPresetExport').addEventListener('click', () => {
      const name = Util.el('selPreset').value;
      const store = State.loadPresetStore();
      const data = { app: 'auto-parking-planner', kind: 'preset', name, preset: store[name] || State.capturePreset() };
      Util.download((name || 'preset').replace(/[^\w-]+/g, '_') + '.json', JSON.stringify(data, null, 2), 'application/json');
    });
    Util.el('filePresetImport').addEventListener('change', e => {
      const file = e.target.files[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = () => {
        try {
          const data = JSON.parse(reader.result);
          const preset = data.preset || data;
          const name = data.name || file.name.replace(/\.json$/i, '');
          if (typeof preset !== 'object') throw new Error('bad');
          const store = State.loadPresetStore();
          store[name] = preset;
          State.savePresetStore(store);
          this.refreshPresetSelect(name);
          this.hint(`Preset “${name}” imported.`);
        } catch (err) {
          this.hint('Import failed: the file is not a valid preset JSON.');
        }
        e.target.value = '';
      };
      reader.readAsText(file);
    });

    /* Parcel import (GeoJSON / coordinate list) */
    Util.el('btnParcelImport').addEventListener('click', () => {
      this.importParcelText(Util.el('parcelPaste').value);
    });
    Util.el('fileParcelImport').addEventListener('change', e => {
      const file = e.target.files[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = () => { this.importParcelText(String(reader.result)); e.target.value = ''; };
      reader.readAsText(file);
    });

    /* Exports */
    Util.el('btnExpSVG').addEventListener('click', () => this.exportSVG());
    Util.el('btnExpPNG').addEventListener('click', () => this.exportPNG());
    Util.el('btnExpPrint').addEventListener('click', () => this.exportPrint());
    Util.el('btnExpCSV').addEventListener('click', () => this.exportCSV());
    Util.el('btnExpJSON').addEventListener('click', () => this.exportProject());
    Util.el('fileProjectImport').addEventListener('change', e => this.importProject(e));
  },

  /**
   * Import a parcel from pasted GeoJSON or a "lat, lng" line list
   * (copied from Suhail, Balady, Google Earth, QGIS…). The geometry is
   * projected to local metres, the boundary + location metadata are set,
   * and the layout regenerates immediately.
   */
  importParcelText(text) {
    text = (text || '').trim();
    if (!text) { this.hint('Paste a GeoJSON geometry or coordinate lines first.'); return; }
    let coords = null;

    /* 1) GeoJSON ([lng, lat] order) */
    try {
      const gj = JSON.parse(text);
      const rings = [];
      const walk = node => {
        if (!node || typeof node !== 'object') return;
        if (node.type === 'FeatureCollection') (node.features || []).forEach(walk);
        else if (node.type === 'Feature') walk(node.geometry);
        else if (node.type === 'GeometryCollection') (node.geometries || []).forEach(walk);
        else if (node.type === 'Polygon') { if (node.coordinates && node.coordinates[0]) rings.push(node.coordinates[0]); }
        else if (node.type === 'MultiPolygon') (node.coordinates || []).forEach(pg => { if (pg[0]) rings.push(pg[0]); });
        else if (node.type === 'LineString') { if (node.coordinates) rings.push(node.coordinates); }
      };
      walk(gj);
      if (rings.length) {
        /* choose the largest ring (a parcel file may carry several) */
        let bestRing = rings[0], bestA = -1;
        for (const r of rings) {
          const a = Math.abs(G.polygonSignedArea(r.map(c => ({ x: c[0], y: c[1] }))));
          if (a > bestA) { bestA = a; bestRing = r; }
        }
        coords = bestRing.map(c => ({ lat: +c[1], lng: +c[0] }));
      }
    } catch (e) { /* not JSON — try coordinate lines */ }

    /* 2) plain "lat, lng" per line */
    if (!coords) {
      const pairs = [];
      for (const line of text.split(/[\n;]+/)) {
        const nums = line.match(/-?\d+(?:\.\d+)?/g);
        if (nums && nums.length >= 2) pairs.push([parseFloat(nums[0]), parseFloat(nums[1])]);
      }
      if (pairs.length >= 3) {
        /* order detection: values beyond ±90 must be longitudes */
        const firstIsLng = pairs.some(p => Math.abs(p[0]) > 90);
        coords = pairs.map(p => firstIsLng ? { lat: p[1], lng: p[0] } : { lat: p[0], lng: p[1] });
      }
    }

    if (!coords || coords.length < 3) {
      this.hint('Import failed: could not read a polygon (need GeoJSON or at least 3 "lat, lng" lines).');
      return;
    }
    if (coords.some(c => !isFinite(c.lat) || !isFinite(c.lng) || Math.abs(c.lat) > 90 || Math.abs(c.lng) > 180)) {
      this.hint('Import failed: coordinates out of range — expected WGS84 latitude/longitude.');
      return;
    }
    /* drop a repeated closing vertex and consecutive duplicates */
    const clean = [];
    for (const c of coords) {
      const prev = clean[clean.length - 1];
      if (!prev || Math.abs(prev.lat - c.lat) > 1e-9 || Math.abs(prev.lng - c.lng) > 1e-9) clean.push(c);
    }
    if (clean.length > 1) {
      const f = clean[0], l = clean[clean.length - 1];
      if (Math.abs(f.lat - l.lat) < 1e-9 && Math.abs(f.lng - l.lng) < 1e-9) clean.pop();
    }
    if (clean.length < 3) { this.hint('Import failed: fewer than 3 distinct vertices.'); return; }

    const { pts, centroid } = G.latLngToLocal(clean);
    const area = G.polygonArea(pts);
    if (!isFinite(area) || area < 30 || area > 5e6) {
      this.hint(`Import failed: computed area ${Util.fmt(area, 0)} m² is outside the plausible parcel range.`);
      return;
    }

    State.pushUndo();
    const s = State.s;
    s.land.mode = 'poly';
    s.land.polygon = pts;
    s.project.lat = Math.round(centroid.lat * 1e6) / 1e6;
    s.project.lng = Math.round(centroid.lng * 1e6) / 1e6;
    /* references to edges that no longer exist would silently misbehave */
    const n = pts.length;
    s.roads = s.roads.filter(r => r.edge < n);
    s.accessPoints = s.accessPoints.filter(a => a.edge < n);
    /* keep the building visible on the new site */
    const c = G.centroid(pts);
    for (const b of s.buildings) { b.x = Math.round(c.x * 10) / 10; b.y = Math.round(c.y * 10) / 10; }
    Interact.polyMode = null;
    Interact.polyDraft = [];
    this.syncFromState();
    App.recalc(true);
    Renderer.zoomFit(App.ctx);
    this.hint(`Parcel imported: ${n} vertices, ${Util.fmtArea(area)} at ${s.project.lat}, ${s.project.lng}. Now assign roads to the correct edges.`);
  },

  finishPolygon() {
    if (Interact.polyMode !== 'draw') return;
    if (Interact.polyDraft.length < 3) {
      this.hint('A boundary needs at least 3 points.');
      return;
    }
    State.pushUndo();
    State.s.land.polygon = Interact.polyDraft.slice();
    Interact.polyDraft = [];
    Interact.polyMode = null;
    Interact.updateCursor();
    App.recalc(true);
    Renderer.zoomFit(App.ctx);
    this.hint('Boundary closed — area ' + Util.fmtArea(G.polygonArea(State.s.land.polygon)) + '.');
  },

  selectedBuilding() {
    const s = State.s;
    s.selectedBuilding = Util.clamp(s.selectedBuilding, 0, Math.max(0, s.buildings.length - 1));
    return s.buildings[s.selectedBuilding] || null;
  },

  setBuildingMethod(m) {
    const b = this.selectedBuilding();
    if (!b) return;
    State.pushUndo();
    b.defMethod = m;
    if (m === 'area') this.applyAreaMethod(b);
    this.syncBuildingFields();
    App.scheduleRecalc();
  },

  /** Derive width/depth from a required area + aspect ratio (§6 method B). */
  applyAreaMethod(b) {
    const area = Math.max(9, b.area || 9);
    const aspect = Util.clamp(b.aspect || 1, 0.2, 5);
    b.width = Math.round(Math.sqrt(area * aspect) * 100) / 100;
    b.depth = Math.round((area / b.width) * 100) / 100;
  },

  alignBuilding(edgeIdx) {
    const b = this.selectedBuilding();
    const ctx = App.ctx;
    if (!b || b.locked || !ctx.valid || edgeIdx >= ctx.edges.length) return;
    State.pushUndo();
    const e = ctx.edges[edgeIdx];
    const n = G.inwardNormal(ctx.landPoly, edgeIdx);
    const rot = G.d2r(b.rotation);
    const ax = G.rot({ x: 1, y: 0 }, rot), ay = G.rot({ x: 0, y: 1 }, rot);
    const halfExt = Math.abs(G.dot(ax, n)) * b.width / 2 + Math.abs(G.dot(ay, n)) * b.depth / 2;
    const inset = ctx.buildingInsets[edgeIdx] + State.s.sidewalks.building;
    const foot = G.closestPointOnSeg({ x: b.x, y: b.y }, e.a, e.b);
    const target = G.add(foot, G.scale(n, inset + halfExt));
    b.x = Math.round(target.x * 100) / 100;
    b.y = Math.round(target.y * 100) / 100;
    this.syncBuildingFields();
    App.recalc(true);
  },

  deleteBuilding() {
    const s = State.s;
    if (!s.buildings.length) return;
    State.pushUndo();
    s.buildings.splice(s.selectedBuilding, 1);
    s.selectedBuilding = Math.max(0, s.selectedBuilding - 1);
    Interact.sel = null;
    Interact.updateSelInfo();
    this.syncBuildingFields();
    App.recalc(true);
  },

  syncBuildingFields() {
    const s = State.s;
    const b = this.selectedBuilding();
    /* tabs */
    const tabs = Util.el('buildingTabs');
    tabs.textContent = '';
    s.buildings.forEach((bb, i) => {
      const chip = document.createElement('button');
      chip.className = 'chip' + (i === s.selectedBuilding ? ' active' : '');
      chip.textContent = bb.name;
      chip.addEventListener('click', () => {
        s.selectedBuilding = i;
        Interact.select({ type: 'building', idx: i });
        this.syncBuildingFields();
      });
      tabs.appendChild(chip);
    });
    const ids = ['bldgWidth', 'bldgDepth', 'bldgArea', 'bldgAspect', 'bldgX', 'bldgY', 'bldgRot', 'bldgLock'];
    for (const id of ids) Util.el(id).disabled = !b;
    if (!b) return;
    Util.el('bldgMethodWh').checked = b.defMethod === 'wh';
    Util.el('bldgMethodArea').checked = b.defMethod === 'area';
    Util.el('bldgWhFields').classList.toggle('is-hidden', b.defMethod !== 'wh');
    Util.el('bldgAreaFields').classList.toggle('is-hidden', b.defMethod !== 'area');
    const setIf = (id, v) => { const el = Util.el(id); if (document.activeElement !== el) el.value = v; };
    setIf('bldgWidth', b.width); setIf('bldgDepth', b.depth);
    setIf('bldgArea', b.area || Math.round(b.width * b.depth)); setIf('bldgAspect', b.aspect || 1.33);
    setIf('bldgX', Util.fmt(b.x, 2)); setIf('bldgY', Util.fmt(b.y, 2)); setIf('bldgRot', Util.fmt(b.rotation, 1));
    Util.el('bldgLock').checked = !!b.locked;
  },

  /* ── dynamic lists ────────────────────────────────────────────── */
  bindDynamicLists() { /* built on demand in build*List() */ },

  edgeOptions(selected) {
    const n = App.ctx.valid ? App.ctx.edges.length : 4;
    let html = '';
    for (let i = 0; i < n; i++) {
      const name = Generator.edgeName(State.s, i, n);
      html += `<option value="${i}" ${i === selected ? 'selected' : ''}>${Util.escapeHTML(name)}</option>`;
    }
    return html;
  },

  buildRoadsList() {
    const wrap = Util.el('roadsList');
    wrap.textContent = '';
    State.s.roads.forEach((r, idx) => {
      const card = document.createElement('div');
      card.className = 'dyn-card' + (r.width < r.minWidth ? ' bad' : '');
      card.innerHTML = `
        <div class="dyn-head"><span>${Util.escapeHTML(r.name)}</span>
          <button class="dyn-del" title="Remove road">✕</button></div>
        <div class="fld-row">
          <label class="fld"><span>Name</span><input type="text" data-f="name" value="${Util.escapeHTML(r.name)}"></label>
          <label class="fld"><span>Edge</span><select data-f="edge">${this.edgeOptions(r.edge)}</select></label>
        </div>
        <div class="fld-row">
          <label class="fld"><span>Width (m)</span><input type="number" data-f="width" min="3" max="100" step="0.5" value="${r.width}"></label>
          <label class="fld"><span>Min width (m)</span><input type="number" data-f="minWidth" min="0" max="100" step="0.5" value="${r.minWidth}"></label>
        </div>
        <div class="fld-row">
          <label class="fld"><span>Classification</span>
            <select data-f="classification">
              ${['local', 'collector', 'arterial', 'highway'].map(c => `<option value="${c}" ${r.classification === c ? 'selected' : ''}>${c}</option>`).join('')}
            </select></label>
          <label class="fld"><span>Designation</span>
            <select data-f="designation">
              ${['front', 'side', 'rear'].map(c => `<option value="${c}" ${r.designation === c ? 'selected' : ''}>${c}</option>`).join('')}
            </select></label>
        </div>
        <div class="fld-row">
          <label class="fld chk"><span>Access allowed</span><input type="checkbox" data-f="accessAllowed" ${r.accessAllowed ? 'checked' : ''}></label>
          <label class="fld"><span>Max entrances</span><input type="number" data-f="maxEntrances" min="0" max="6" step="1" value="${r.maxEntrances}"></label>
        </div>`;
      card.querySelector('.dyn-del').addEventListener('click', () => {
        State.pushUndo();
        State.s.roads = State.s.roads.filter(x => x.id !== r.id);
        this.buildRoadsList();
        App.scheduleRecalc();
      });
      card.querySelectorAll('[data-f]').forEach(inp => {
        inp.addEventListener('change', () => {
          State.pushUndo();
          const f = inp.dataset.f;
          let v;
          if (inp.type === 'checkbox') v = inp.checked;
          else if (inp.type === 'number' || f === 'edge') {
            v = parseFloat(inp.value);
            if (isNaN(v)) return;
            if (f === 'edge' || f === 'maxEntrances') v = Math.round(v);
          } else v = inp.value;
          r[f] = v;
          App.scheduleRecalc();
          if (f === 'name' || f === 'width' || f === 'minWidth') this.buildRoadsList();
        });
      });
      wrap.appendChild(card);
    });
    if (!State.s.roads.length) {
      wrap.innerHTML = '<p class="hint">No roads defined yet.</p>';
    }
  },

  buildAccessList() {
    const wrap = Util.el('accessList');
    wrap.textContent = '';
    State.s.accessPoints.forEach((ap, idx) => {
      const card = document.createElement('div');
      card.className = 'dyn-card';
      const edgeLen = App.ctx.valid && ap.edge < App.ctx.edges.length ? App.ctx.edges[ap.edge].len : 100;
      card.innerHTML = `
        <div class="dyn-head"><span>Access ${idx + 1} — ${ap.type === 'both' ? 'entry + exit' : ap.type === 'in' ? 'entry' : 'exit'}</span>
          <button class="dyn-del" title="Remove access point">✕</button></div>
        <div class="fld-row">
          <label class="fld"><span>Type</span>
            <select data-f="type">
              <option value="both" ${ap.type === 'both' ? 'selected' : ''}>Combined in/out</option>
              <option value="in" ${ap.type === 'in' ? 'selected' : ''}>Entrance only</option>
              <option value="out" ${ap.type === 'out' ? 'selected' : ''}>Exit only</option>
            </select></label>
          <label class="fld"><span>Width (m)</span><input type="number" data-f="width" min="3" max="15" step="0.5" value="${ap.width}"></label>
        </div>
        <div class="fld-row">
          <label class="fld"><span>Edge</span><select data-f="edge">${this.edgeOptions(ap.edge)}</select></label>
          <label class="fld"><span>Position (${Util.fmt(ap.t * edgeLen, 1)} m along)</span>
            <input type="range" data-f="t" min="0.02" max="0.98" step="0.01" value="${ap.t}"></label>
        </div>`;
      card.querySelector('.dyn-del').addEventListener('click', () => {
        State.pushUndo();
        /* remove by identity — a stale card must never delete a different point */
        State.s.accessPoints = State.s.accessPoints.filter(a => a.id !== ap.id);
        if (Interact.sel && Interact.sel.type === 'ap' && Interact.sel.id === ap.id) Interact.select(null);
        this.buildAccessList();
        App.scheduleRecalc();
      });
      card.querySelectorAll('[data-f]').forEach(inp => {
        inp.addEventListener('change', () => {
          State.pushUndo();
          const f = inp.dataset.f;
          let v = inp.type === 'range' || inp.type === 'number' || f === 'edge' ? parseFloat(inp.value) : inp.value;
          if (f === 'edge') v = Math.round(v);
          ap[f] = v;
          App.scheduleRecalc();
          if (f !== 't') this.buildAccessList();
        });
      });
      wrap.appendChild(card);
    });
    if (!State.s.accessPoints.length) {
      wrap.innerHTML = '<p class="hint">No access points. Use the button below, then click an eligible edge on the plan.</p>';
    }
  },

  buildMixedList() {
    const wrap = Util.el('mixedRowsList');
    wrap.textContent = '';
    const uses = ['Bank branch', 'Office', 'Retail', 'Customer-service area', 'ATM area', 'Restaurant', 'Storage', 'Other'];
    State.s.demand.mixed.forEach((row, idx) => {
      const req = row.ratio > 0 ? Math.ceil(row.area / row.ratio) : 0;
      const card = document.createElement('div');
      card.className = 'dyn-card';
      card.innerHTML = `
        <div class="dyn-head"><span>${Util.escapeHTML(row.use)} — ${req} space(s)</span>
          <button class="dyn-del" title="Remove use">✕</button></div>
        <div class="fld-row">
          <label class="fld"><span>Use type</span>
            <select data-f="use">${uses.map(u => `<option value="${u}" ${row.use === u ? 'selected' : ''}>${u}</option>`).join('')}</select></label>
        </div>
        <div class="fld-row">
          <label class="fld"><span>Area (m²)</span><input type="number" data-f="area" min="0" step="1" value="${row.area}"></label>
          <label class="fld"><span>m² per space</span><input type="number" data-f="ratio" min="1" step="1" value="${row.ratio}"></label>
        </div>`;
      card.querySelector('.dyn-del').addEventListener('click', () => {
        State.pushUndo();
        State.s.demand.mixed = State.s.demand.mixed.filter(x => x !== row);
        this.buildMixedList();
        App.scheduleRecalc();
      });
      card.querySelectorAll('[data-f]').forEach(inp => {
        inp.addEventListener('change', () => {
          State.pushUndo();
          const f = inp.dataset.f;
          const v = inp.type === 'number' ? parseFloat(inp.value) : inp.value;
          if (inp.type === 'number' && isNaN(v)) return;
          row[f] = v;
          this.buildMixedList();
          App.scheduleRecalc();
        });
      });
      wrap.appendChild(card);
    });
  },

  refreshPresetSelect(selectName) {
    const sel = Util.el('selPreset');
    const store = State.loadPresetStore();
    sel.textContent = '';
    for (const name of Object.keys(store)) {
      const opt = document.createElement('option');
      opt.value = name;
      opt.textContent = name;
      sel.appendChild(opt);
    }
    if (selectName && store[selectName]) sel.value = selectName;
  },

  /* ── option cards ─────────────────────────────────────────────── */
  updateOptionCards() {
    const wrap = Util.el('optionCards');
    wrap.textContent = '';
    const opts = App.options || {};
    const s = State.s;
    for (const key of ['A', 'B', 'C']) {
      const o = opts[key];
      const card = document.createElement('div');
      card.className = 'option-card' + (s.optimization.option === key ? ' active' : '');
      if (!o) {
        card.innerHTML = `<div><div class="oc-name">Option ${key}</div><div class="oc-sub">not available</div></div>`;
      } else {
        const lsPct = App.ctx.landArea > 0 ? o.stats.landscapeArea / App.ctx.landArea * 100 : 0;
        card.innerHTML = `
          <div>
            <div class="oc-name">Option ${key} — ${o.label}</div>
            <div class="oc-sub">${o.meta.geom.label} rows @ ${Util.fmt(o.meta.orient, 0)}° · landscape ${Util.fmt(lsPct, 0)}%</div>
          </div>
          <div style="text-align:right">
            <div class="oc-count">${o.stats.total}</div>
            <div class="oc-score">score ${Util.fmt(o.score, 0)}</div>
          </div>`;
        card.addEventListener('click', () => {
          State.pushUndo();
          s.optimization.option = key;
          App.recalc(false);
          this.updateOptionCards();
        });
      }
      wrap.appendChild(card);
    }
  },

  /* ── results: KPIs, warnings, summary ─────────────────────────── */
  updateResults() {
    const s = State.s, ctx = App.ctx, layout = App.layout, demand = App.demandInfo, warn = App.warnings;
    const st = layout ? layout.stats : null;
    const set = (id, v, cls) => {
      const el = Util.el(id);
      el.textContent = v;
      el.classList.remove('good', 'bad');
      if (cls) el.classList.add(cls);
    };
    const footprint = Demand.footprintArea(s);
    const coverage = ctx.landArea > 0 ? footprint / ctx.landArea * 100 : 0;
    set('kpiLandArea', Util.fmtArea(ctx.landArea));
    set('kpiFootprint', Util.fmtArea(footprint));
    set('kpiCoverage', Util.fmt(coverage, 1) + ' %', coverage > s.regs.maxCoveragePct ? 'bad' : null);
    set('kpiRequired', String(demand.required));
    set('kpiProvided', st ? String(st.total) : '–', st && st.total >= demand.required ? 'good' : 'bad');
    const bal = st ? st.total - demand.required : 0;
    set('kpiBalance', st ? (bal >= 0 ? '+' + bal : String(bal)) : '–', bal >= 0 ? 'good' : 'bad');
    set('kpiRegular', st ? String(st.regular) : '–');
    set('kpiAccessible', st ? String(st.accessible) : '–');
    set('kpiEV', st ? String(st.ev) : '–');
    set('kpiEfficiency', st && st.total > 0 ? Util.fmt(st.paved / st.total, 1) : '–');
    set('kpiLandscape', st ? Util.fmtArea(st.landscapeArea) + ' (' + Util.fmt(ctx.landArea ? st.landscapeArea / ctx.landArea * 100 : 0, 0) + '%)' : '–');
    set('kpiPaved', st ? Util.fmtArea(st.paved) : '–');

    Util.el('landAreaInline').textContent = Util.fmtArea(ctx.landArea);
    Util.el('requiredInline').textContent = String(demand.required);
    Util.el('polyVertexCount').textContent = String(s.land.mode === 'poly' ? (Interact.polyMode === 'draw' ? Interact.polyDraft.length : s.land.polygon.length) : 0);
    Util.el('polyArea').textContent = s.land.mode === 'poly' && s.land.polygon.length >= 3 ? Util.fmtArea(G.polygonArea(s.land.polygon)) : '–';

    /* banner */
    const banner = Util.el('compBanner');
    banner.classList.remove('ok', 'warn', 'bad');
    if (warn.status === 'ok') { banner.classList.add('ok'); banner.textContent = '✓ Compliant — all checks passed'; }
    else if (warn.status === 'warn') { banner.classList.add('warn'); banner.textContent = '⚠ Review — ' + warn.items.length + ' warning(s)'; }
    else {
      const bads = warn.items.filter(i => i.level === 'bad').length;
      banner.classList.add('bad'); banner.textContent = '✕ Non-compliant — ' + bads + ' violation(s)';
    }

    /* warnings list */
    const ul = Util.el('warnList');
    ul.textContent = '';
    if (!warn.items.length) {
      const li = document.createElement('li');
      li.className = 'ok';
      li.textContent = 'No warnings — the layout satisfies all configured checks.';
      ul.appendChild(li);
    } else {
      for (const it of warn.items) {
        const li = document.createElement('li');
        if (it.level === 'bad') li.className = 'bad';
        li.textContent = it.msg;
        ul.appendChild(li);
      }
    }

    this.updateSummaryTable(demand, st, coverage);
    this.updateDesignSummary(demand, st, coverage);
  },

  updateSummaryTable(demand, st, coverage) {
    const s = State.s, ctx = App.ctx;
    const rows = [];
    const pill = ok => ok === true ? '<span class="pill ok">OK</span>' : ok === false ? '<span class="pill bad">FAIL</span>' : '<span class="pill warn">CHECK</span>';
    const needAcc = st && st.total > 0 ? Math.max(s.accessible.min, Math.ceil(st.total / Math.max(1, s.accessible.ratioPer))) : 0;
    const needEV = st ? Math.ceil(st.total * Util.clamp(s.ev.pct, 0, 100) / 100) : 0;
    rows.push(['Parking spaces', String(demand.required), st ? String(st.total) : '–', st ? st.total >= demand.required : false]);
    rows.push(['Accessible parking', String(needAcc), st ? String(st.accessible) : '–', st ? st.accessible >= needAcc : false]);
    rows.push(['EV parking', String(needEV), st ? String(st.ev) : '–', st ? st.ev >= needEV : false]);
    const sbOk = ctx.buildingInfos.every(i => i.violations.length === 0);
    rows.push(['Setbacks',
      `front ${s.setbacks.front} / rear ${s.setbacks.rear} / left ${s.setbacks.left} / right ${s.setbacks.right} m`,
      sbOk ? 'satisfied' : 'violated', sbOk]);
    if (s.roads.length) {
      const roadsOk = s.roads.every(r => r.width >= r.minWidth);
      rows.push(['Road width', s.roads.map(r => '≥' + Util.fmt(r.minWidth)).join(', ') + ' m', s.roads.map(r => Util.fmt(r.width)).join(', ') + ' m', roadsOk]);
    }
    rows.push(['Site coverage', '≤ ' + Util.fmt(s.regs.maxCoveragePct) + ' %', Util.fmt(coverage, 1) + ' %', coverage <= s.regs.maxCoveragePct]);
    const lsPct = st && ctx.landArea > 0 ? st.landscapeArea / ctx.landArea * 100 : 0;
    rows.push(['Landscape', '≥ ' + Util.fmt(s.regs.minLandscapePct) + ' %', Util.fmt(lsPct, 1) + ' %', lsPct >= s.regs.minLandscapePct]);
    const apOk = ctx.apInfos.length > 0 && ctx.apInfos.every(a => a.road && a.road.accessAllowed && a.cornerDist >= s.accessRules.minCornerDist);
    rows.push(['Access points', '≥ 1, corner ≥ ' + Util.fmt(s.accessRules.minCornerDist) + ' m', String(ctx.apInfos.length), apOk]);
    if (App.layout) {
      const g = App.layout.meta.geom;
      rows.push(['Drive aisle', (g.oneWay ? 'one-way' : 'two-way') + ' standard', Util.fmt(g.aisle) + ' m', g.aisle >= (g.oneWay ? Rules.FLOORS.oneWayAisle : Rules.FLOORS.twoWayAisle)]);
    }
    Util.el('sumBody').innerHTML = rows.map(r =>
      `<tr><td>${r[0]}</td><td>${r[1]}</td><td>${r[2]}</td><td>${pill(r[3])}</td></tr>`).join('');
  },

  updateDesignSummary(demand, st, coverage) {
    const s = State.s;
    if (!st) { Util.el('designSummary').textContent = 'Define a land boundary to generate a layout.'; return; }
    const diff = st.total - demand.required;
    const sbOk = App.ctx.buildingInfos.every(i => i.violations.length === 0);
    const g = App.layout.meta.geom;
    const parts = [];
    parts.push(`The proposed scheme provides ${st.total} parking space${st.total === 1 ? '' : 's'} against a requirement of ${demand.required}, resulting in a ${diff >= 0 ? 'surplus' : 'deficit'} of ${Math.abs(diff)} space${Math.abs(diff) === 1 ? '' : 's'}.`);
    parts.push(`The building footprint occupies ${Util.fmt(coverage, 1)}% of the ${Util.fmt(App.ctx.landArea, 0)} m² site.`);
    parts.push(sbOk ? 'All mandatory setbacks are satisfied.' : 'One or more mandatory setbacks are violated — see warnings.');
    parts.push(`${st.accessible} accessible space${st.accessible === 1 ? '' : 's'} and ${st.ev} EV-ready space${st.ev === 1 ? '' : 's'} are included.`);
    parts.push(`The layout uses ${g.label} ${App.layout.bands.some(b => b.double) ? 'double-loaded' : 'single-loaded'} rows oriented at ${Util.fmt(App.layout.meta.orient, 0)}° with ${Util.fmt(g.aisle)} m ${g.oneWay ? 'one-way' : 'two-way'} aisles; landscape covers ${Util.fmt(App.ctx.landArea ? st.landscapeArea / App.ctx.landArea * 100 : 0, 0)}% of the site.`);
    Util.el('designSummary').textContent = parts.join(' ');
  },

  /* ── exports (§18) ────────────────────────────────────────────── */

  /** Build a standalone SVG document string of the current plan. */
  buildExportSVG() {
    const s = State.s, ctx = App.ctx;
    const W = 1400, H = 900, TB = 170;      // plan area + title block
    const saved = Util.deepClone(Renderer.view);
    /* fit the site into the export area */
    if (ctx.valid) {
      let bb = G.bbox(ctx.landPoly);
      let margin = 6;
      for (const r of s.roads) margin = Math.max(margin, r.width + 4);
      bb = { minX: bb.minX - margin, minY: bb.minY - margin, maxX: bb.maxX + margin, maxY: bb.maxY + margin };
      const k = Math.min((W - 40) / (bb.maxX - bb.minX), (H - 40) / (bb.maxY - bb.minY));
      Renderer.view.k = k;
      Renderer.view.tx = (W - (bb.maxX - bb.minX) * k) / 2 - bb.minX * k;
      Renderer.view.ty = (H - (bb.maxY - bb.minY) * k) / 2 - bb.minY * k;
    }
    Renderer.applyView();
    App.renderOnly();

    const clone = Renderer.svg.cloneNode(true);
    /* strip interactive layers + screen overlay */
    for (const id of ['ly_handles', 'ly_temp', 'overlay']) {
      const el = clone.querySelector('#' + id);
      if (el) el.remove();
    }
    clone.setAttribute('width', W);
    clone.setAttribute('height', H + TB);
    clone.setAttribute('viewBox', `0 0 ${W} ${H + TB}`);
    clone.setAttribute('xmlns', 'http://www.w3.org/2000/svg');

    const bg = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
    Util.attr(bg, { x: 0, y: 0, width: W, height: H + TB, fill: '#ffffff' });
    clone.insertBefore(bg, clone.firstChild);

    /* title block */
    const st = App.layout ? App.layout.stats : null;
    const demand = App.demandInfo;
    const tb = document.createElementNS('http://www.w3.org/2000/svg', 'g');
    const txt = (x, y, str, size, weight, anchor) => {
      const t = document.createElementNS('http://www.w3.org/2000/svg', 'text');
      Util.attr(t, { x, y, 'font-size': size, 'font-family': 'Arial, sans-serif', fill: '#1f2937', 'font-weight': weight || 400, 'text-anchor': anchor || 'start' });
      t.textContent = str;
      tb.appendChild(t);
    };
    const line = (x1, y1, x2, y2) => {
      const l = document.createElementNS('http://www.w3.org/2000/svg', 'line');
      Util.attr(l, { x1, y1, x2, y2, stroke: '#1f2937', 'stroke-width': 1 });
      tb.appendChild(l);
    };
    const frame = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
    Util.attr(frame, { x: 6, y: 6, width: W - 12, height: H + TB - 12, fill: 'none', stroke: '#1f2937', 'stroke-width': 2 });
    tb.appendChild(frame);
    line(6, H, W - 6, H);
    const y0 = H + 30;
    txt(24, y0, s.project.name || 'Untitled Project', 22, 700);
    txt(24, y0 + 24, `Project no. ${s.project.number || '—'}   ·   Date ${Util.todayISO()}   ·   Designer ${s.project.designer || '—'}`, 12);
    txt(24, y0 + 44, `Land area ${Util.fmt(ctx.landArea, 0)} m²   ·   Building footprint ${Util.fmt(Demand.footprintArea(s), 0)} m²   ·   Scale ≈ 1:${Math.round(96 / 25.4 * 1000 / Renderer.view.k)}`, 12);
    if (st) {
      txt(24, y0 + 64, `Parking: required ${demand.required} · provided ${st.total} (${st.regular} regular, ${st.accessible} accessible, ${st.ev} EV) · ${st.total >= demand.required ? 'COMPLIANT' : 'DEFICIT ' + (demand.required - st.total)}`, 12, 700);
    }
    txt(24, y0 + 92, 'Early-stage planning and feasibility tool. Final compliance must be verified against the applicable municipality, Saudi Building Code, accessibility requirements, Civil Defense requirements, and project-specific authority approvals.', 9);
    /* north arrow */
    const gN = document.createElementNS('http://www.w3.org/2000/svg', 'g');
    Util.attr(gN, { transform: `translate(${W - 70},${y0 + 30}) rotate(${s.land.northAngle || 0})` });
    const nPath = document.createElementNS('http://www.w3.org/2000/svg', 'path');
    Util.attr(nPath, { d: 'M0,-24 L9,12 L0,4 L-9,12 Z', fill: '#334155' });
    gN.appendChild(nPath);
    const nT = document.createElementNS('http://www.w3.org/2000/svg', 'text');
    Util.attr(nT, { x: 0, y: -30, 'text-anchor': 'middle', 'font-size': 14, 'font-weight': 700, fill: '#334155', 'font-family': 'Arial, sans-serif' });
    nT.textContent = 'N';
    gN.appendChild(nT);
    tb.appendChild(gN);
    /* scale bar: 10 m */
    const sbLen = 10 * Renderer.view.k;
    const gS = document.createElementNS('http://www.w3.org/2000/svg', 'g');
    Util.attr(gS, { transform: `translate(${W - 140 - sbLen},${y0 + 50})` });
    const r1 = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
    Util.attr(r1, { x: 0, y: 0, width: sbLen / 2, height: 7, fill: '#1f2937' });
    const r2 = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
    Util.attr(r2, { x: sbLen / 2, y: 0, width: sbLen / 2, height: 7, fill: '#fff', stroke: '#1f2937' });
    gS.appendChild(r1); gS.appendChild(r2);
    const sT = document.createElementNS('http://www.w3.org/2000/svg', 'text');
    Util.attr(sT, { x: sbLen + 8, y: 8, 'font-size': 11, 'font-family': 'Arial, sans-serif', fill: '#1f2937' });
    sT.textContent = '10 m';
    gS.appendChild(sT);
    tb.appendChild(gS);
    clone.appendChild(tb);

    const out = new XMLSerializer().serializeToString(clone);

    /* restore the on-screen view */
    Renderer.view = saved;
    Renderer.applyView();
    App.renderOnly();
    return '<?xml version="1.0" encoding="UTF-8"?>\n' + out;
  },

  exportName(ext) {
    return (State.s.project.name || 'parking-plan').replace(/[^\w؀-ۿ-]+/g, '_') + '.' + ext;
  },

  exportSVG() {
    Util.download(this.exportName('svg'), this.buildExportSVG(), 'image/svg+xml');
    this.hint('SVG exported.');
  },

  exportPNG() {
    const svgStr = this.buildExportSVG();
    const blob = new Blob([svgStr], { type: 'image/svg+xml;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const img = new Image();
    img.onload = () => {
      const canvas = document.createElement('canvas');
      canvas.width = img.width * 2;
      canvas.height = img.height * 2;
      const cx = canvas.getContext('2d');
      cx.fillStyle = '#ffffff';
      cx.fillRect(0, 0, canvas.width, canvas.height);
      cx.drawImage(img, 0, 0, canvas.width, canvas.height);
      URL.revokeObjectURL(url);
      canvas.toBlob(b => {
        if (b) Util.download(this.exportName('png'), b, 'image/png');
        this.hint('PNG exported.');
      }, 'image/png');
    };
    img.onerror = () => { URL.revokeObjectURL(url); this.hint('PNG export failed in this browser — use Export SVG instead.'); };
    img.src = url;
  },

  exportPrint() {
    const svgStr = this.buildExportSVG();
    const st = App.layout ? App.layout.stats : null;
    const w = window.open('', '_blank');
    if (!w) { this.hint('Pop-up blocked — allow pop-ups to print.'); return; }
    w.document.write(`<!DOCTYPE html><html><head><title>${Util.escapeHTML(State.s.project.name)} — Parking Plan</title>
      <style>body{font-family:Arial,sans-serif;margin:16px} svg{width:100%;height:auto;border:1px solid #ccc}
      table{border-collapse:collapse;font-size:12px;margin-top:12px} td,th{border:1px solid #999;padding:4px 8px}</style>
      </head><body>${svgStr}`);
    if (st) {
      w.document.write(`<table><tr><th>Required</th><th>Provided</th><th>Regular</th><th>Accessible</th><th>EV</th></tr>
        <tr><td>${App.demandInfo.required}</td><td>${st.total}</td><td>${st.regular}</td><td>${st.accessible}</td><td>${st.ev}</td></tr></table>`);
    }
    w.document.write('</body></html>');
    w.document.close();
    w.focus();
    setTimeout(() => w.print(), 300);
  },

  exportCSV() {
    const layout = App.layout;
    if (!layout) { this.hint('No layout to export yet.'); return; }
    const rows = [['ID', 'Type', 'Band', 'Row', 'Slot', 'Centre X (m)', 'Centre Y (m)', 'Orientation (deg)', 'Width (m)', 'Length (m)', 'Connected']];
    let aa = 0;
    for (const st of layout.stalls) {
      /* dimensions are measured from the drawn polygon so the schedule
         always matches the plan (accessible width is provided via the
         adjacent hatched access-aisle slot, not a wider stall) */
      const e01 = G.dist(st.poly[0], st.poly[1]), e12 = G.dist(st.poly[1], st.poly[2]);
      rows.push([
        /* IDs match the numbers drawn on the plan */
        st.num != null ? 'P' + String(st.num).padStart(3, '0') : 'AA' + String(++aa).padStart(2, '0'),
        st.type, st.band === -2 ? 'frontage' : st.band >= 0 ? st.band + 1 : 'manual', st.row >= 0 ? st.row + 1 : '-', st.slot >= 0 ? st.slot + 1 : '-',
        st.cx.toFixed(2), st.cy.toFixed(2), Util.fmt(((st.axisWorld % 360) + 360) % 360, 1),
        Util.fmt(Math.min(e01, e12), 2),
        Util.fmt(Math.max(e01, e12), 2),
        st.connected ? 'yes' : 'no'
      ]);
    }
    const csv = rows.map(r => r.map(c => /[",\n]/.test(String(c)) ? '"' + String(c).replace(/"/g, '""') + '"' : c).join(',')).join('\n');
    Util.download(this.exportName('csv'), csv, 'text/csv');
    this.hint('Parking schedule exported (' + (rows.length - 1) + ' rows).');
  },

  exportProject() {
    const data = {
      app: 'auto-parking-planner',
      kind: 'project',
      version: State.s.version,
      savedAt: new Date().toISOString(),
      state: State.s,
      results: App.layout ? {
        option: State.s.optimization.option,
        stats: App.layout.stats,
        meta: {
          orient: App.layout.meta.orient, stallKey: App.layout.meta.stallKey,
          flip: App.layout.meta.flip, extraInset: App.layout.meta.extraInset
        },
        alternatives: ['A', 'B', 'C'].map(k => {
          const o = App.options && App.options[k];
          return o ? { option: k, label: o.label, total: o.stats.total, score: o.score, orient: o.meta.orient, stallKey: o.meta.stallKey } : null;
        })
      } : null
    };
    Util.download(this.exportName('json'), JSON.stringify(data, null, 2), 'application/json');
    this.hint('Project JSON exported.');
  },

  importProject(e) {
    const file = e.target.files[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = () => {
      const prevJson = JSON.stringify(State.s);
      try {
        const data = JSON.parse(reader.result);
        const incoming = data.state || data;
        const migrated = State.migrate(incoming);
        if (!migrated) throw new Error('unrecognised file');
        /* trial-run the imported state BEFORE committing it */
        Generator.buildContext(migrated);
        Demand.compute(migrated);
        State.pushUndo();
        State.s = migrated;
        this.syncFromState();
        App.recalc(true);
        Renderer.zoomFit(App.ctx);
        this.hint('Project imported.');
      } catch (err) {
        /* roll back to the pre-import state on any failure */
        try {
          State.s = JSON.parse(prevJson);
          this.syncFromState();
          App.recalc(true);
        } catch (e2) { /* keep current state */ }
        this.hint('Import failed: the file is not a valid project JSON. The previous project was kept.');
      }
      e.target.value = '';
    };
    reader.readAsText(file);
  }
};

/* ═══════════════════════ 10. App — bootstrap + pipeline ═══════════════════════ */

const App = {
  /* Safe stub until the first buildContext() so UI code that renders
     before the first recalc (e.g. dynamic lists) never sees null. */
  ctx: { valid: false, edges: [], insets: [], landArea: 0, buildingInfos: [], apInfos: [], spines: [], throats: [], obstaclesStall: [], obstaclesAisle: [], entrancePt: null },
  options: null,      // { A, B, C } generated alternatives
  layout: null,       // active layout (with manual edits applied)
  demandInfo: { required: 0, baseRequired: 0, gfa: 0, breakdown: [] },
  warnings: { items: [], status: 'ok' },
  _previewQueued: false,

  state() { return State.s; },

  init() {
    State.init();
    Renderer.init(Util.el('plan'));
    Interact.init(Util.el('plan'));
    this.ctx = Generator.buildContext(State.s);
    UI.init();
    this.recalc(true);
    Renderer.zoomFit(this.ctx);
    Interact.setTool('select');
    window.addEventListener('resize', () => {
      Renderer.applyView();
      Renderer.drawOverlay();
    });
  },

  /**
   * The regeneration pipeline (§12).
   *  full=true  → rebuild context, run the complete optimizer (all
   *               orientations × angles × packing directions × options).
   *  full=false → rebuild context and regenerate only the active option's
   *               winning combination (used for live previews and manual
   *               edits — fast enough to run every frame).
   */
  recalc(full) {
    const s = State.s;
    this.demandInfo = Demand.compute(s);
    this.ctx = Generator.buildContext(s);

    if (this.ctx.valid) {
      if (full || !this.options) {
        this.options = Generator.optimize(s, this.ctx, this.demandInfo);
      }
      let base = this.options[s.optimization.option] || this.options.B || this.options.A;
      if (base) {
        /* regenerate fresh so manual edits never stack up on a stale copy */
        this.layout = Generator.regenerateLike(s, this.ctx, this.demandInfo, base.meta);
        Generator.applyManualEdits(this.layout, s, this.ctx);
      } else {
        this.layout = null;
      }
    } else {
      this.options = null;
      this.layout = null;
    }

    this.warnings = Rules.evaluate(s, this.ctx, this.layout, this.demandInfo);
    this.renderOnly();
    UI.updateResults();
    UI.refreshAccessList();
    if (full) UI.updateOptionCards();
    State.autosave();
  },

  /** Debounced full recalculation for input changes (§12). */
  scheduleRecalc: Util.debounce(function () { App.recalc(true); }, 160),

  /** Lightweight preview while dragging: one candidate, rAF-throttled. */
  schedulePreview() {
    if (this._previewQueued) return;
    this._previewQueued = true;
    requestAnimationFrame(() => {
      this._previewQueued = false;
      this.recalc(false);
    });
  },

  /** Redraw without regenerating (layer toggles, selection changes…). */
  renderOnly() {
    Renderer.render(State.s, this.ctx || { valid: false }, this.layout, this.demandInfo);
    Interact.renderTempOverlays();
  }
};

/* Boot when the DOM is ready — robust whether this script runs during
   parsing or is injected after the document has already loaded. */
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', () => App.init());
} else {
  App.init();
}
