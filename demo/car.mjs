// Geometry of the mecanum test-bench car, shared by build.mjs (Node + audit)
// and the live page (inlined with exports stripped; k, G, T are passed in).
// Car frame: x forward, y left, z up, origin on the floor under the body
// centre. Pose = { x, y, th }: translate, then rotate the car frame about z.
// Audit shapes are closures (A) => A.xxx(...), built at the CURRENT pose;
// the browser never calls them.

export const M = {
  H: 36, W: 39,            // half wheelbase (x), half track (y)
  axleZ: 15.6,             // axle height = rolling envelope radius
  Rf: 12,                  // roller mount radius (flange)
  rRoller: 3.6, barrelHalf: 5.0, N: 9, stagger: 1.5,
  flangeIn: 4.45, flangeOut: 5.65, hubR: 3.4, flangeR: 13.6,
  plate: { x: -80, y: -31, w: 160, d: 62, r: 4 },
  plateZ: 38, plateH: 3,
  standoffZ: 41, deckZ: 53, deckH: 3,
  VMAX: 150, WMAX: 0.9,    // command limits: mm/s and rad/s
};

// PDF numbering: 1 rear-left, 2 front-left, 3 front-right, 4 rear-right.
// hand +1: bottom-contact roller axis (t + y)/sqrt2; hand -1: (t - y)/sqrt2.
export const WHEELS = [
  { i: 1, x: -M.H, y: M.W, hand: -1 },
  { i: 2, x: M.H, y: M.W, hand: 1 },
  { i: 3, x: M.H, y: -M.W, hand: -1 },
  { i: 4, x: -M.H, y: -M.W, hand: 1 },
];

// Inverse kinematics: wheel rim speeds from the body twist (mm/s, rad/s).
export const ik = (vX, vY, w) => {
  const s = M.H + M.W;
  return [0, vX + vY - s * w, vX - vY - s * w, vX + vY + s * w, vX - vY + s * w];
};

// Idle tour: [vX, vY, w, seconds]
export const TOUR = [
  [M.VMAX, 0, 0, 1.8], [0, 0, 0, 0.8],
  [0, M.VMAX, 0, 1.8], [0, 0, 0, 0.8],
  [0, 0, 0.8, 2.4], [0, 0, 0, 0.8],
  [M.VMAX * 0.6, M.VMAX * 0.6, 0, 1.6], [0, 0, 0, 1.0],
];

export function makeCar(k, G, T, P) {
  const V = G.viewOf(P);
  const S = (paths, style) => k.solidSvg(paths, style);
  const Ln = (d, style) => k.lineSvg(d, style);
  const depth = (p) => G.dot3(p, V);
  let Q = 1; // quality 0..1, lowered by the page when frames run slow

  const pose = { x: 0, y: 0, th: 0 };
  const setPose = (x, y, th) => { pose.x = x; pose.y = y; pose.th = th; };
  const cs = () => [Math.cos(pose.th), Math.sin(pose.th)];
  const X = ([x, y, z]) => { const [c, s] = cs(); return [pose.x + x * c - y * s, pose.y + x * s + y * c, z]; };
  const X2 = ([x, y]) => { const [c, s] = cs(); return [pose.x + x * c - y * s, pose.y + x * s + y * c]; };
  const Xd = ([x, y, z]) => { const [c, s] = cs(); return [x * c - y * s, x * s + y * c, z]; };
  const XF = (F) => G.frameOf(X(F.o), G.unit3(Xd(F.a)), G.unit3(Xd(F.u)), G.unit3(Xd(F.v)));
  const rotY = ([x, y, z], a) => {
    const c = Math.cos(a), s = Math.sin(a);
    return [x * c + z * s, y, -x * s + z * c];
  };
  const isoOf = (p) => k.iso(p, P);
  const faceMatrix = (o3, u3, v3) => {
    const o = isoOf(o3), eu = isoOf(G.add3(o3, u3)), ev = isoOf(G.add3(o3, v3));
    const u = [eu[0] - o[0], eu[1] - o[1]], v = [ev[0] - o[0], ev[1] - o[1]];
    const f = (n) => Math.round(n * 1000) / 1000;
    return `matrix(${f(u[0])} ${f(u[1])} ${f(v[0])} ${f(v[1])} ${f(o[0])} ${f(o[1])})`;
  };
  // Small text flat on the floor / deck, reading along the car's x or y axis.
  const flatText = (at3, along, text, size, tone, anchor = "middle") => {
    const [c, s] = cs();
    const u = along === "x" ? [c, s, 0] : [-s, c, 0];
    const v = along === "x" ? [-s, c, 0] : [-c, -s, 0];
    return k.faceTextSvg(faceMatrix(at3, u, v), text, { size, tone, anchor });
  };
  // Filled arrowhead on a horizontal plane, tip at at3, tail along d2.
  const head = (at3, d2, tone, size = 4.2) => {
    const p0 = X(at3), p1 = G.add3(p0, Xd([d2[0], d2[1], 0]));
    const tip = isoOf(p0), out = isoOf(p1);
    let ux = out[0] - tip[0], uy = out[1] - tip[1];
    const l = Math.hypot(ux, uy) || 1; ux /= l; uy /= l;
    const px = -uy, py = ux;
    const d = `M${tip[0].toFixed(1)} ${tip[1].toFixed(1)}L${(tip[0] - ux * size + px * size * 0.42).toFixed(1)} ${(tip[1] - uy * size + py * size * 0.42).toFixed(1)}L${(tip[0] - ux * size - px * size * 0.42).toFixed(1)} ${(tip[1] - uy * size - py * size * 0.42).toFixed(1)}Z`;
    return `<path d="${d}" fill="var(--anatomy-${tone})" stroke="none"/>`;
  };
  const headZ = (at3, tone, size = 4) => {
    const tip = isoOf(X(at3));
    const d = `M${tip[0].toFixed(1)} ${tip[1].toFixed(1)}L${(tip[0] - size * 0.62).toFixed(1)} ${(tip[1] + size).toFixed(1)}L${(tip[0] + size * 0.62).toFixed(1)} ${(tip[1] + size).toFixed(1)}Z`;
    return `<path d="${d}" fill="var(--anatomy-${tone})" stroke="none"/>`;
  };
  const halo = (ring, blur = 5) =>
    `<path class="iso-halo" style="filter:blur(${blur}px)" d="${k.extrude(ring, 0.05, 0.2, P).fill}"/>`;

  // ---- posed primitives ---------------------------------------------------
  const slab = (plan, z, h, style, steps = 8, bevel = 0) =>
    ({ svg: S(k.extrude(k.ringOf(plan, steps).map(X2), z, h, P, { bevel }), style), shape: (A) => A.extruded(k.ringOf(plan, steps).map(X2), z, h) });
  const cyl = (cx, cy, r, z, h, style, steps = 20, bevel = 0) => {
    const [wx, wy] = X2([cx, cy]);
    return { svg: S(k.cylinder(wx, wy, r, z, h, P, steps, bevel), style), shape: (A) => A.cylinder(...X2([cx, cy]), r, z, h) };
  };
  const outline = (plan, z, tone, steps = 8) =>
    ({ svg: Ln(k.onTop(k.ringOf(plan, steps).map(X2), z, P, true), { tone }), shape: null });
  const lineTop = (a, b, z, tone, extra = {}) =>
    ({ svg: Ln(k.onTop([X2(a), X2(b)], z, P), { tone, ...extra }), shape: null });
  const arcTop = (cx, cy, r, a0, a1, z, tone) => {
    const pts = [];
    const n = 16;
    for (let i = 0; i <= n; i++) {
      const a = a0 + (a1 - a0) * i / n;
      pts.push(isoOf([...X2([cx + r * Math.cos(a), cy + r * Math.sin(a)]), z]));
    }
    return { svg: Ln(k.pathOf(pts), { tone }), shape: null };
  };

  // ---- wheels -------------------------------------------------------------
  const rollerProf = (sLo, sHi) => [
    [sLo, 0.8], [-5.55, 0.8], [-5.55, 1.3], [-5.0, 2.0], [-4.1, 2.88, 1],
    [-2.5, 3.42, 1], [0, M.rRoller, 1], [2.5, 3.42, 1], [4.1, 2.88, 1],
    [5.0, 2.0], [5.55, 1.3], [5.55, 0.8], [sHi, 0.8],
  ];
  const rollerLocal = (w, index) => {
    const phi = (-90 + index * (360 / M.N)) * Math.PI / 180;
    const c = [M.Rf * Math.cos(phi), 0, M.Rf * Math.sin(phi)];
    const t = [-Math.sin(phi), 0, Math.cos(phi)];
    const axis = G.unit3([t[0], t[1] + w.hand, t[2]]);
    const o = index % 2 ? M.stagger : -M.stagger;
    return { axis, center: G.add3(c, G.mul3(axis, o)) };
  };
  const axleFrame = (w) => G.frameAlong([w.x, w.y, M.axleZ], [0, 1, 0]);

  // One wheel in three depth chunks: pre (stub + far flange),
  // roll (hub + rollers, depth-sorted), post (near flange + cap [+ stub]).
  // rollOnly skips the static chunks (per-frame roller refresh).
  function wheelItem(w, phase, rollOnly = false) {
    const steps = Math.max(8, Math.round(12 * Q));
    const map = (p) => X([w.x + p[0], w.y + p[1], M.axleZ + p[2]]);
    const inB = -Math.sign(w.y), outB = -inB;
    const F = XF(axleFrame(w));
    const style = { tone: "hi", crease: "lo" };
    const pre = [], roll = [], post = [];
    const add = (list, paths, st, shape, at) => list.push({ svg: S(paths, st), shape, at });

    if (!rollOnly) {
      const stubS = inB > 0 ? [M.flangeIn, M.flangeIn + 1.4] : [-M.flangeIn - 1.4, -M.flangeIn];
      add(pre, G.disc(stubS[0], stubS[1], 2.2, F, P, { steps: 14 }), { tone: "hi" },
        (A) => A.disc(F, stubS[0], stubS[1], 2.2));
      add(pre, G.ringBand(M.flangeIn, M.flangeOut, M.hubR, M.flangeR, F, P, { steps: 40 }), style,
        (A) => A.ring(F, M.flangeIn, M.flangeOut, M.hubR, M.flangeR));
    }

    add(roll, G.disc(-M.flangeOut, M.flangeOut, M.hubR, F, P, { steps: 24 }), { tone: "hi" },
      (A) => A.disc(F, -M.flangeOut, M.flangeOut, M.hubR), depth(map([0, 0, 0])));
    for (let index = 0; index < M.N; index++) {
      const { axis, center } = rollerLocal(w, index);
      const c2 = rotY(center, phase), a2 = rotY(axis, phase);
      const cy = c2[1], ay = a2[1];
      const sA = (-6.0 - cy) / ay, sB = (6.0 - cy) / ay;
      const prof = rollerProf(Math.min(sA, sB), Math.max(sA, sB));
      const RF = XF(G.frameAlong(G.add3([w.x, w.y, M.axleZ], c2), a2));
      add(roll, G.lathe(prof, RF, P, { steps }), { tone: "hi" },
        (A) => A.solid(RF, prof), depth(map(c2)));
    }
    roll.sort((a, b) => a.at - b.at);

    if (!rollOnly) {
      add(post, G.ringBand(-M.flangeOut, -M.flangeIn, M.hubR, M.flangeR, F, P, { steps: 40 }), style,
        (A) => A.ring(F, -M.flangeOut, -M.flangeIn, M.hubR, M.flangeR));
      add(post, G.disc(outB * M.flangeOut, outB * (M.flangeOut + 0.8), 2.5, F, P, { steps: 14 }), { tone: "hi" },
        (A) => A.disc(F, outB * M.flangeOut, outB * (M.flangeOut + 0.8), 2.5));
      if (inB < 0) {
        add(post, G.disc(-M.flangeIn - 1.4, -M.flangeIn, 2.2, F, P, { steps: 14 }), { tone: "hi" },
          (A) => A.disc(F, -M.flangeIn - 1.4, -M.flangeIn, 2.2));
      }
    }
    const svg = (list) => list.map((p) => p.svg).join("");
    return {
      name: `wheel${w.i}`, at: X([w.x, w.y, M.axleZ]),
      shapes: [...pre, ...roll, ...post].map((p) => p.shape).filter(Boolean),
      pre: svg(pre), post: svg(post), roll: svg(roll),
    };
  }

  // ---- chassis ------------------------------------------------------------
  function plateItem() {
    const parts = [slab(M.plate, M.plateZ, M.plateH, { tone: "mid" }, 8, 0.8)];
    parts.push(outline(k.insetPlan(M.plate, 4), M.plateZ + M.plateH, "lo"));
    for (const [sx, sy] of k.corners(M.plate, 14)) parts.push(cyl(sx, sy, 1.1, M.plateZ + M.plateH, 0.7, { tone: "mid" }, 12));
    const tray = { x: -60, y: 16, w: 30, d: 14, r: 3 };
    parts.push(slab(tray, 26, 12, { tone: "lo" }, 6, 0.5));
    parts.push(outline(k.insetPlan(tray, 2.4), M.plateZ, "lo"));
    return { name: "plate", at: X([0, 0, M.plateZ]), shapes: parts.map((p) => p.shape).filter(Boolean), svg: parts.map((p) => p.svg).join("") };
  }
  function boardUnderItem() {
    const plan = { x: -25, y: -10, w: 50, d: 20, r: 2 };
    const parts = [slab(plan, 34.4, 2.2, { tone: "mid" }, 6, 0.4)];
    for (const px of [-20, 20]) for (const py of [-6, 6]) parts.push(cyl(px, py, 1.2, 36.6, 1.4, { tone: "mid" }, 10));
    parts.push(slab({ x: -12, y: -5, w: 16, d: 10, r: 1 }, 36.6, 1.2, { tone: "lo" }, 4));
    for (const sx of [-16, 16]) for (const sy of [-1, 1]) {
      const F = XF(G.frameAlong([sx, sy * 10, 35.8], [0, sy, 0]));
      parts.push({ svg: S(G.disc(0, 1.6, 1.0, F, P, { steps: 12 }), { tone: "mid" }), shape: (A) => A.disc(G.frameAlong([sx, sy * 10, 35.8], [0, sy, 0]), 0, 1.6, 1.0) });
    }
    return { name: "boardU", at: X([0, 0, 35]), shapes: parts.map((p) => p.shape).filter(Boolean), svg: parts.map((p) => p.svg).join("") };
  }
  function deckItem() {
    const parts = [];
    for (const [sx, sy] of [[-72, -20], [72, -20], [-72, 20], [72, 20], [0, -24], [0, 24]])
      parts.push(cyl(sx, sy, 2.2, M.standoffZ, M.deckZ - M.standoffZ, { tone: "mid" }, 14));
    parts.push(slab(M.plate, M.deckZ, M.deckH, { tone: "mid" }, 8, 0.8));
    parts.push(outline(k.insetPlan(M.plate, 4), M.deckZ + M.deckH, "lo"));
    for (const [sx, sy] of k.corners(M.plate, 14)) parts.push(cyl(sx, sy, 1.1, M.deckZ + M.deckH, 0.7, { tone: "mid" }, 12));
    return { name: "deck", at: X([0, 0, M.deckZ]), shapes: parts.map((p) => p.shape).filter(Boolean), svg: parts.map((p) => p.svg).join("") };
  }
  function boardTopItem() {
    const plan = { x: -60, y: -9, w: 32, d: 18, r: 2 };
    const parts = [slab(plan, M.deckZ + M.deckH, 2, { tone: "mid" }, 6, 0.4)];
    parts.push(slab({ x: -52, y: -4.5, w: 11, d: 9, r: 1 }, M.deckZ + M.deckH + 2, 1, { tone: "lo" }, 4));
    return { name: "boardT", at: X([-44, 0, M.deckZ]), shapes: parts.map((p) => p.shape).filter(Boolean), svg: parts.map((p) => p.svg).join("") };
  }
  function antennaItem() {
    const parts = [cyl(16, 6, 0.8, M.deckZ + M.deckH, 11, { tone: "mid" }, 10)];
    return { name: "antenna", at: X([16, 6, M.deckZ]), shapes: parts.map((p) => p.shape).filter(Boolean), svg: parts.map((p) => p.svg).join("") };
  }
  function labelItem() {
    const plan = { x: 20, y: -28, w: 26, d: 7, r: 1.4 };
    const parts = [slab(plan, M.deckZ + M.deckH, 0.8, { tone: "lo" }, 4)];
    parts.push(outline(k.insetPlan(plan, 1.2), M.deckZ + M.deckH + 0.8, "lo"));
    return { name: "label", at: X([33, -24.5, M.deckZ]), shapes: parts.map((p) => p.shape).filter(Boolean), svg: parts.map((p) => p.svg).join("") };
  }
  function triadItem() {
    const z = M.deckZ + M.deckH + 0.15;
    const parts = [];
    parts.push({ svg: Ln(k.onTop(k.circleRing(...X2([0, 0]), 2.5, 20), z, P, true), { tone: "red" }), shape: null });
    parts.push(lineTop([3.2, 0], [23, 0], z, "red"));
    parts.push(head([25.4, 0, z], [1, 0], "red"));
    parts.push(lineTop([0, 3.2], [0, 23], z, "red"));
    parts.push(head([0, 25.4, z], [0, 1], "red"));
    parts.push(cyl(0, 0, 1.3, z, 2.6, { tone: "mid" }, 12));
    parts.push({ svg: Ln(k.pathOf([isoOf(X([0, 0, z + 2.4])), isoOf(X([0, 0, z + 21]))]), { tone: "red" }), shape: (A) => A.tube([X([0, 0, z + 2.4]), X([0, 0, z + 24.6])], 0.35) });
    parts.push(headZ([0, 0, z + 25.4], "red"));
    const shapes = [
      (A) => A.tube([X([3.2, 0, z]), X([21, 0, z])], 0.35),
      (A) => A.tube([X([0, 3.2, z]), X([0, 21, z])], 0.35),
      (A) => A.cylinder(...X2([0, 0]), 1.3, z, 2.6),
      (A) => A.tube([X([0, 0, z + 2.4]), X([0, 0, z + 24.6])], 0.35),
      (A) => A.disc(G.frameOf(X([25.4, 0, z + 0.05]), [0, 0, 1], [1, 0, 0], [0, 1, 0]), 0, 0.1, 1.8),
      (A) => A.disc(G.frameOf(X([0, 25.4, z + 0.05]), [0, 0, 1], [1, 0, 0], [0, 1, 0]), 0, 0.1, 1.8),
      (A) => A.ball(X([0, 0, z + 25.4]), 1.4),
    ];
    return { name: "triad", at: X([0, 0, z]), shapes, svg: parts.map((p) => p.svg).join("") };
  }

  // ---- drive train per wheel ---------------------------------------------
  function driveItems(w) {
    const inB = -Math.sign(w.y);
    const yb = (a, b) => (inB > 0 ? [w.y + a, w.y + b] : [w.y - b, w.y - a]);
    const items = [];
    {
      const [y0, y1] = yb(M.flangeIn + 1.4, M.flangeIn + 11.0);
      const plan = { x: w.x - 5, y: y0, w: 10, d: y1 - y0, r: 1.5 };
      const parts = [slab(plan, 12, 11, { tone: "mid" }, 4, 0.5)];
      items.push({ name: `gb${w.i}`, at: X([w.x, (y0 + y1) / 2, 17]), shapes: parts.map((p) => p.shape).filter(Boolean), svg: parts.map((p) => p.svg).join("") });
    }
    let gland = null;
    {
      const [y0, y1] = yb(M.flangeIn + 11.0, M.flangeIn + 25.0);
      const axis = [0, inB, 0];
      const o = inB > 0 ? [w.x, y0, M.axleZ] : [w.x, y1, M.axleZ];
      const F = XF(G.frameAlong(o, axis));
      const prof = [[0, 4.5], [0.9, 4.5], [1.6, 3.6, 1], [12.4, 3.6, 1], [13.2, 4.5], [14, 4.5]];
      const seam = G.arcOf(F, 6.4, 3.6, P, { steps: 20 });
      const gy = inB > 0 ? y1 - 2.2 : y0 + 2.2;
      gland = [w.x, gy, M.axleZ + 6.0];
      const parts = [
        { svg: S(G.solidOf(prof, F, P, { steps: 20 }), { tone: "mid", inner: Ln(seam, { tone: "faint" }) }), shape: (A) => A.solid(F, prof) },
        cyl(w.x, gy, 1.15, M.axleZ + 4.3, 1.7, { tone: "mid" }, 12),
      ];
      items.push({ name: `can${w.i}`, at: X([w.x, (y0 + y1) / 2, M.axleZ]), shapes: parts.map((p) => p.shape).filter(Boolean), svg: parts.map((p) => p.svg).join("") });
    }
    {
      const [y0, y1] = yb(M.flangeIn + 25.0, M.flangeIn + 28.0);
      const plan = { x: w.x - 6, y: y0, w: 12, d: y1 - y0, r: 1.5 };
      const parts = [slab(plan, 12, M.plateZ - 12, { tone: "mid" }, 4, 0.5)];
      items.push({ name: `br${w.i}`, at: X([w.x, (y0 + y1) / 2, 27]), shapes: parts.map((p) => p.shape).filter(Boolean), svg: parts.map((p) => p.svg).join("") });
    }
    return { items, gland };
  }

  // ---- motor cables -------------------------------------------------------
  function cableItem(w, gland) {
    const side = Math.sign(w.y);
    const lane = side * 13.2;
    const edgeY = side * 11.6;
    const edgeX = w.x > 0 ? 16 : -16;
    const ptsCar = [
      gland,
      [w.x, gland[1], gland[2] + 6.5],
      [w.x * 0.82, lane, 31],
      [edgeX, lane * 0.94, 34.0],
      [edgeX, edgeY - side * 1.5, 35.8],
      [edgeX, edgeY, 35.8],
    ];
    const route = G.fillet(ptsCar.map(X), 2.6, 5, { P, tube: 0.6 });
    const pieces = T.tubePieces(route, 0.6, P, { maxLength: 300 });
    const svg = pieces.map((piece) => T.tubeSvg(piece, { tone: "mid" })).join("");
    return {
      name: `cable${w.i}`, at: route[Math.floor(route.length / 2)], svg, route, r: 0.6,
      shape: (A) => A.tube(route, 0.6, { limp: true }),
    };
  }

  // ---- live marks ---------------------------------------------------------
  // Floor paint in the body frame: shadow, scribes, stations, wheel-3
  // decomposition, per-wheel rim-speed arrows.
  function marksFloor(cmd) {
    const z = 0.15;
    const p = [];
    p.push(halo(k.ringOf(k.insetPlan(M.plate, -7), 8).map(X2)));
    for (const w of WHEELS) p.push(halo(k.circleRing(...X2([w.x, w.y]), 16.5, 20), 4));
    p.push(lineTop([-92, 0], [92, 0], z, "faint"));
    p.push(lineTop([0, -92], [0, 92], z, "faint"));
    for (const sx of [-M.H, M.H]) p.push(lineTop([sx, -56], [sx, 56], z, "faint"));
    p.push(flatText([95, 7.5, z], "x", "H", 3.4, "mid"));
    p.push(flatText([8.5, -91, z], "y", "W", 3.4, "mid"));
    for (const w of WHEELS) {
      p.push(lineTop([w.x - 3, w.y], [w.x + 3, w.y], z, "mid"));
      p.push(lineTop([w.x, w.y - 3], [w.x, w.y + 3], z, "mid"));
      p.push(flatText([w.x, w.y + Math.sign(w.y) * 12.5, z], "x", String(w.i), 4.2, "mid"));
    }
    // wheel 3 station: alpha (centre direction vs -x) and beta (roller vs x),
    // drawn outside the wheel silhouette so nothing hides under the tyre
    const w3 = WHEELS[2], c = [w3.x, w3.y];
    const toO = G.unit3([-w3.x, -w3.y, 0]);
    p.push(lineTop([c[0] + toO[0] * 14, c[1] + toO[1] * 14], [c[0] + toO[0] * 30, c[1] + toO[1] * 30], z, "lo"));
    p.push(lineTop([c[0] - 21, c[1]], [c[0] + 21, c[1]], z, "lo"));
    const rollDir = [0.7071, -0.7071];
    const c2 = [c[0] + rollDir[0] * 17, c[1] + rollDir[1] * 17];
    p.push(lineTop(c, [c2[0] + rollDir[0] * 9, c2[1] + rollDir[1] * 9], z, "lo"));
    p.push(lineTop([c2[0] - 9, c2[1]], [c2[0] + 9, c2[1]], z, "lo"));
    const aA = Math.atan2(toO[1], toO[0]);
    p.push(arcTop(c[0], c[1], 15.5, aA, Math.PI, z, "mid"));
    p.push(flatText([c[0] + 19.5 * Math.cos((aA + Math.PI) / 2), c[1] + 19.5 * Math.sin((aA + Math.PI) / 2), z], "x", "α3", 3.4, "mid"));
    p.push(arcTop(c2[0], c2[1], 7.5, -Math.PI / 4, 0, z, "mid"));
    p.push(flatText([c2[0] + 11.5 * Math.cos(-Math.PI / 8), c2[1] + 11.5 * Math.sin(-Math.PI / 8), z], "x", "β3", 3.4, "mid"));
    // decomposition at wheel 3: v_c = viX·x̂ + vir·ĉ  (ĉ = (x̂−ŷ)/√2, hand B)
    const [vX, vY, w] = cmd;
    const vcx = vX - w * w3.y, vcy = vY + w * w3.x;
    const vir = -vcy * Math.SQRT2;                 // component along the roller axis
    const vr = 3.5 + Math.abs(vir) / 630 * 22;
    const vs = Math.sign(vir) || 1;
    p.push(lineTop(c2, [c2[0] + vs * vr * rollDir[0], c2[1] + vs * vr * rollDir[1]], z, "lit"));
    p.push(head([c2[0] + vs * (vr + 3.6) * rollDir[0], c2[1] + vs * (vr + 3.6) * rollDir[1]], [vs * rollDir[0], vs * rollDir[1]], "lit", 3.6));
    p.push(flatText([c2[0] + vs * (vr + 9.5) * rollDir[0], c2[1] + vs * (vr + 9.5) * rollDir[1] - 2, z], "x", "v3r", 3, "mid"));
    // rim-speed arrow at every station
    const vi = ik(vX, vY, w);
    for (const wl of WHEELS) {
      const v = vi[wl.i];
      if (Math.abs(v) < 3) continue;
      const len = 4 + Math.abs(v) / 630 * 18, s = Math.sign(v);
      p.push(lineTop([wl.x + s * 15, wl.y], [wl.x + s * (15 + len), wl.y], z, "red"));
      p.push(head([wl.x + s * (18.5 + len), wl.y], [s, 0], "red", 4));
    }
    const frame = XF(G.frameOf([0, 0, 0.1], [0, 0, 1], [1, 0, 0], [0, 1, 0]));
    return { name: "marksF", at: X([0, 0, 0.1]), svg: p.map((e) => (typeof e === "string" ? e : e.svg)).join(""), shapes: [(A) => A.disc(frame, 0, 0.2, 96)] };
  }

  // Deck-top command vector: body velocity arrow + spin arc.
  function marksDeck(cmd) {
    const z = M.deckZ + M.deckH + 0.15;
    const p = [];
    const [vX, vY, w] = cmd;
    const mag = Math.hypot(vX, vY);
    if (mag > 3) {
      const len = 3 + mag / (M.VMAX * Math.SQRT2) * 18;
      const d = [vX / mag, vY / mag];
      p.push(lineTop([d[0] * 3, d[1] * 3], [d[0] * len, d[1] * len], z, "red"));
      p.push(head([d[0] * (len + 3.8), d[1] * (len + 3.8), z], d, "red", 3.8));
    }
    if (Math.abs(w) > 0.02) {
      const s = Math.sign(w), r = 8.5;
      const a0 = -Math.PI / 2 - s * 0.55, a1 = -Math.PI / 2 + s * 0.55;
      p.push(arcTop(0, 0, r, a0, a1, z, "red"));
      const tang = [-Math.sin(a1) * s, Math.cos(a1) * s];
      p.push(head([r * Math.cos(a1), r * Math.sin(a1), z], tang, "red", 3.6));
    }
    const frame = XF(G.frameOf([0, 0, z], [0, 0, 1], [1, 0, 0], [0, 1, 0]));
    return { name: "marksD", at: X([0, 0, z]), svg: p.map((e) => (typeof e === "string" ? e : e.svg)).join(""), shapes: [(A) => A.disc(frame, 0, 0.15, 28)] };
  }

  // The full ordered item list of the car at a pose (translation is applied
  // by the page's group transform, so items are built at (0, 0, th)).
  function collectItems(phases, cmd, th = 0) {
    setPose(0, 0, th);
    const list = [marksFloor(cmd)];
    const glands = [];
    for (const w of WHEELS) {
      const d = driveItems(w);
      list.push(...d.items);
      glands.push(d.gland);
    }
    list.push(plateItem(), boardUnderItem(), deckItem(), boardTopItem(),
      antennaItem(), labelItem(), triadItem());
    WHEELS.forEach((w, i) => {
      const wheel = wheelItem(w, phases[i]);
      list.push({ ...wheel, svg: wheel.pre + `<g data-part="roll${w.i}">${wheel.roll}</g>` + wheel.post });
    });
    WHEELS.forEach((w, i) => list.push(cableItem(w, glands[i])));
    list.push(marksDeck(cmd));
    return list;
  }

  return {
    M, WHEELS, pose, setPose, X, X2, Xd, XF, depth, isoOf,
    slab, cyl, outline, lineTop, arcTop, head, headZ, flatText, halo,
    wheelItem, plateItem, boardUnderItem, deckItem, boardTopItem, antennaItem,
    labelItem, triadItem, driveItems, cableItem, marksFloor, marksDeck,
    rollerLocal, axleFrame, collectItems, setQuality: (q) => { Q = q; },
  };
}
