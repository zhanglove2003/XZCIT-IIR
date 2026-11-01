const D = __DATA__;
const P = D.P;
const CAR = makeCar(K, G, T, P);
const CB = makeConsole(K, G, P, { CS: D.CS, JX: D.JX, JY: D.JY, WMAX: M.WMAX });
const still = matchMedia("(prefers-reduced-motion: reduce)");
const stage = document.querySelector(".iso-stage");
const svg = stage.querySelector("svg");
const $ = (sel) => svg.querySelector(sel);
const readout = document.querySelector("[data-readout]");
const at = (p) => K.iso(p, P);

const carG = $('[data-part="car"]');
const itemEls = {};
for (const g of svg.querySelectorAll("[data-item]")) itemEls[g.dataset.item] = g;
const rollEls = {};
for (let i = 1; i <= 4; i++) rollEls[i] = $(`[data-part="roll${i}"]`);
const stickEl = $('[data-part="stick"]'), ringEl = $('[data-part="ringptr"]'), barsEl = $('[data-part="bars"]');
const buttonEls = {};
for (const g of svg.querySelectorAll("[data-button]")) buttonEls[g.dataset.button] = g;

const ex = K.axisVector(P, [1, 0, 0]), ey = K.axisVector(P, [0, 1, 0]);
const det = ex[0] * ey[1] - ex[1] * ey[0];
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const TAU = Math.PI * 2;

const st = {
  pose: { x: 0, y: 0, th: 0 },
  cmd: [0, 0, 0], target: [0, 0, 0],
  keys: { fwd: 0, back: 0, left: 0, right: 0, q: 0, e: 0 },
  hold: null, drag: null,
  phases: [0, 0, 0, 0],
  lastTh: -9, lastBody: 0, lastCmd: "\u0000",
  touring: true, tourT: 0, idle: 0,
  visible: false, raf: 0, lastT: 0, bodyMs: 0, quality: 1,
};

const TOUR_TOTAL = TOUR.reduce((s, t) => s + t[3], 0);
function tourCmd(t) {
  let acc = 0;
  for (const [vX, vY, w, secs] of TOUR) {
    if (t < acc + secs) return [vX, vY, w];
    acc += secs;
  }
  return [0, 0, 0];
}
function keyTarget() {
  const k = st.keys;
  return [
    clamp((k.fwd ? M.VMAX : 0) - (k.back ? M.VMAX : 0) + (st.hold === "fwd" ? M.VMAX : 0), -M.VMAX, M.VMAX),
    clamp((k.left ? M.VMAX : 0) - (k.right ? M.VMAX : 0) + (st.hold === "strafe" ? M.VMAX : 0), -M.VMAX, M.VMAX),
    clamp((k.q ? M.WMAX : 0) - (k.e ? M.WMAX : 0) + (st.hold === "spin" ? M.WMAX : 0), -M.WMAX, M.WMAX),
  ];
}
const vi = () => ik(st.cmd[0], st.cmd[1], st.cmd[2]);

function wake() {
  st.touring = false;
  st.idle = 0;
}
function renderConsole() {
  const mag = Math.hypot(st.cmd[0], st.cmd[1]);
  const sc = mag > M.VMAX ? M.VMAX / mag : 1;
  stickEl.innerHTML = CB.stick(st.cmd[0] * sc / M.VMAX, st.cmd[1] * sc / M.VMAX);
  ringEl.innerHTML = CB.ringPtr(st.cmd[2]);
  barsEl.innerHTML = CB.bars(vi());
  const v = vi();
  const fmt = (n) => (n >= 0 ? "+" : "−") + Math.abs(Math.round(n));
  const deg = ((st.pose.th * 180 / Math.PI) % 360 + 360) % 360;
  readout.textContent =
    `v = (${fmt(st.cmd[0])}, ${fmt(st.cmd[1])}, ${st.cmd[2].toFixed(2)}) · ` +
    `v1..v4 = ${fmt(v[1])} ${fmt(v[2])} ${fmt(v[3])} ${fmt(v[4])} mm/s · ` +
    `x ${Math.round(st.pose.x)} y ${Math.round(st.pose.y)} θ ${deg.toFixed(0)}°`;
}
function rebuildBody(th) {
  const items = CAR.collectItems(st.phases, st.cmd, th);
  const norm = ((th % TAU) + TAU) % TAU;
  const order = D.ORDERS[Math.round(norm / (D.STEP * Math.PI / 180)) % D.ORDERS.length];
  const byName = new Map(items.map((i) => [i.name, i]));
  for (const name of order) {
    const el = itemEls[name], it = byName.get(name);
    if (el && it && el.innerHTML !== it.svg) el.innerHTML = it.svg;
  }
}
function rebuildMarks() {
  CAR.setPose(0, 0, st.pose.th);
  itemEls.marksF.innerHTML = CAR.marksFloor(st.cmd).svg;
  itemEls.marksD.innerHTML = CAR.marksDeck(st.cmd).svg;
}
function render(now, force) {
  const { x, y, th } = st.pose;
  carG.setAttribute("transform", `translate(${(x * ex[0] + y * ey[0]).toFixed(2)} ${(x * ex[1] + y * ey[1]).toFixed(2)})`);
  const thChanged = Math.abs(th - st.lastTh) > 0.0035;
  const cmdKey = st.cmd.map((v) => v.toFixed(1)).join(",");
  const nowMs = performance.now();
  const interval = st.quality < 0.5 ? 90 : st.quality < 1 ? 62 : 45;
  if ((thChanged && nowMs - st.lastBody > interval) || force === "body") {
    const t0 = performance.now();
    rebuildBody(th);
    st.bodyMs = st.bodyMs * 0.6 + (performance.now() - t0) * 0.4;
    if (st.bodyMs > 26 && st.quality > 0.45) { st.quality -= 0.25; CAR.setQuality(st.quality); }
    st.lastTh = th; st.lastBody = nowMs; st.lastCmd = cmdKey;
  } else if (cmdKey !== st.lastCmd || force === "marks") {
    rebuildMarks();
    st.lastCmd = cmdKey;
  }
  for (let i = 1; i <= 4; i++) {
    const roll = CAR.wheelItem(WHEELS[i - 1], st.phases[i - 1], true).roll;
    if (rollEls[i].dataset.roll !== roll) {
      rollEls[i].innerHTML = roll;
      rollEls[i].dataset.roll = roll;
    }
  }
  renderConsole();
}
function tick(now) {
  const dt = st.lastT ? Math.min((now - st.lastT) / 1000, 1 / 30) : 1 / 60;
  st.lastT = now;
  if (st.touring) { st.tourT += dt; st.target = tourCmd(st.tourT % TOUR_TOTAL); }
  else {
    st.idle += dt;
    if (st.idle > 9 && !still.matches) { st.touring = true; st.tourT = 0; }
  }
  const calm = still.matches;
  for (let i = 0; i < 3; i++) {
    st.cmd[i] = calm ? st.target[i] : st.cmd[i] + (st.target[i] - st.cmd[i]) * (1 - Math.exp(-dt / 0.22));
    if (Math.abs(st.target[i] - st.cmd[i]) < 0.02) st.cmd[i] = st.target[i];
  }
  const v = ik(st.cmd[0], st.cmd[1], st.cmd[2]);
  if (!calm) {
    const [c, s] = [Math.cos(st.pose.th), Math.sin(st.pose.th)];
    st.pose.x = clamp(st.pose.x + (st.cmd[0] * c - st.cmd[1] * s) * dt, -D.CLAMP.x, D.CLAMP.x);
    st.pose.y = clamp(st.pose.y + (st.cmd[0] * s + st.cmd[1] * c) * dt, -D.CLAMP.y, D.CLAMP.y);
    const cdx = st.pose.x - D.CS.x, cdy = st.pose.y - D.CS.y;
    const cd = Math.hypot(cdx, cdy);
    if (cd < 140) {   // keep the car's body clear of the console
      st.pose.x = D.CS.x + cdx / cd * 140;
      st.pose.y = D.CS.y + cdy / cd * 140;
    }
    st.pose.th += st.cmd[2] * dt;
    for (let i = 0; i < 4; i++) st.phases[i] += v[i + 1] / M.axleZ * dt;
  }
  render(now);
  const busy = st.touring || st.drag || st.hold ||
    st.cmd.some((cv, i) => Math.abs(cv - st.target[i]) > 0.02) ||
    (!calm && (v.some((cv) => Math.abs(cv) > 0.5) || Math.abs(st.target[2]) > 0.01));
  st.raf = st.visible && busy ? requestAnimationFrame(tick) : 0;
}
const run = () => { if (!st.raf && st.visible) { st.lastT = 0; st.raf = requestAnimationFrame(tick); } };

// ---------------------------------------------------------------- input ----
const WIDTH_SVG = svg.viewBox.baseVal.width;
function svgPoint(event) {
  const box = svg.getBoundingClientRect();
  const scale = box.width / WIDTH_SVG;
  return [(event.clientX - box.left) / scale, (event.clientY - box.top) / scale];
}
const stickBase = at([D.JX, D.JY, 26]);
const ringC = at([D.JX, D.JY, 18]);
stage.addEventListener("pointerdown", (event) => {
  const [mx, my] = svgPoint(event);
  if (Math.hypot(mx - stickBase[0], my - stickBase[1]) < 26) {
    st.drag = { kind: "stick", ox: mx, oy: my };
    stage.setPointerCapture(event.pointerId);
  } else if (Math.hypot(mx - ringC[0], my - ringC[1]) < 15) {
    st.drag = { kind: "ring", a0: Math.atan2(my - ringC[1], mx - ringC[0]), w0: st.target[2] };
    stage.setPointerCapture(event.pointerId);
  }
  wake(); run();
  event.preventDefault();
});
stage.addEventListener("pointermove", (event) => {
  if (!st.drag) return;
  const [mx, my] = svgPoint(event);
  if (st.drag.kind === "stick") {
    const dx = mx - st.drag.ox, dy = my - st.drag.oy;
    let ux = (dx * ey[1] - dy * ey[0]) / det, uy = (dy * ex[0] - dx * ex[1]) / det;
    const mag = Math.hypot(ux, uy);
    if (mag > 1) { ux /= mag; uy /= mag; }
    st.target = [ux * M.VMAX, uy * M.VMAX, st.target[2]];
  } else {
    const a = Math.atan2(my - ringC[1], mx - ringC[0]);
    let dA = a - st.drag.a0;
    while (dA > Math.PI) dA -= TAU;
    while (dA < -Math.PI) dA += TAU;
    st.target = [st.target[0], st.target[1], clamp(st.drag.w0 + dA / 4.712, -1, 1) * M.WMAX];
  }
  wake(); run();
});
const endDrag = (event) => {
  if (!st.drag) return;
  if (st.drag.kind === "stick") st.target = [0, 0, st.target[2]];
  else st.target = [st.target[0], st.target[1], 0];
  st.drag = null;
  wake(); run();
};
stage.addEventListener("pointerup", endDrag);
stage.addEventListener("pointercancel", endDrag);

const PRESET = { fwd: [M.VMAX, 0, 0], strafe: [0, M.VMAX, 0], spin: [0, 0, M.WMAX], stop: [0, 0, 0] };
for (const [name, el] of Object.entries(buttonEls)) {
  const bx = D.CS.x - 31 + ["fwd", "strafe", "spin", "stop"].indexOf(name) * 12.5;
  el.addEventListener("pointerdown", (event) => {
    event.stopPropagation();
    el.innerHTML = CB.button(bx, D.CS.y + 14, 0.7);
    if (name === "stop") { st.hold = null; st.target = [0, 0, 0]; st.keys = { fwd: 0, back: 0, left: 0, right: 0, q: 0, e: 0 }; }
    else { st.hold = name; st.target = PRESET[name].slice(); }
    wake(); run();
  });
  const up = () => {
    el.innerHTML = CB.button(bx, D.CS.y + 14, 0);
    if (st.hold === name) { st.hold = null; st.target = [0, 0, 0]; }
    run();
  };
  el.addEventListener("pointerup", up);
  el.addEventListener("pointerleave", up);
}

const KEYMAP = { ArrowUp: "fwd", ArrowDown: "back", ArrowLeft: "left", ArrowRight: "right", q: "q", Q: "q", e: "e", E: "e" };
stage.addEventListener("keydown", (event) => {
  if (event.key === " ") {
    event.preventDefault();
    st.target = [0, 0, 0]; st.keys = { fwd: 0, back: 0, left: 0, right: 0, q: 0, e: 0 };
    wake(); run();
    return;
  }
  const key = KEYMAP[event.key];
  if (!key) return;
  event.preventDefault();
  st.keys[key] = 1;
  st.target = keyTarget();
  wake(); run();
});
stage.addEventListener("keyup", (event) => {
  const key = KEYMAP[event.key];
  if (!key) return;
  st.keys[key] = 0;
  st.target = keyTarget();
  run();
});

new IntersectionObserver((entries) => {
  st.visible = entries[entries.length - 1].isIntersecting;
  if (st.visible) run();
}, { rootMargin: "120px 0px" }).observe(stage);
if (still.matches) st.touring = false;

render(0, "body");
window.__mk = (vx, vy, w) => {
  st.touring = false;
  st.target = [vx, vy, w];
  st.cmd = [vx, vy, w];
  rebuildMarks();
  renderConsole();
  for (let i = 1; i <= 4; i++) {
    const roll = CAR.wheelItem(WHEELS[i - 1], st.phases[i - 1], true).roll;
    rollEls[i].innerHTML = roll;
  }
  run();
};
