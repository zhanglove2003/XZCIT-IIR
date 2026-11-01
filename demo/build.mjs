import { readFileSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import * as k from "/home/snow/.agents/skills/anatomy/kit/iso-kit.mjs";
import * as G from "/home/snow/.agents/skills/anatomy/kit/lathe.mjs";
import * as T from "/home/snow/.agents/skills/anatomy/kit/tube.mjs";
import * as A from "/home/snow/.agents/skills/anatomy/kit/audit.mjs";
import { kitScript } from "/home/snow/.agents/skills/anatomy/scripts/inline-kit.mjs";
import { M, WHEELS, makeCar } from "./car.mjs";
import { makeConsole } from "./console.mjs";

const HERE = dirname(fileURLToPath(import.meta.url));
const KIT = "/home/snow/.agents/skills/anatomy/kit";
const theme = process.argv.includes("--light") ? "light" : "dark";
const AUDIT = process.argv.includes("--audit");

const WIDTH = 800, HEIGHT = 560;
const FLOOR = { x: -270, y: -180, w: 540, d: 360, r: 10 };
const BOUND = { x: -232, y: -128, w: 464, d: 256, r: 20 };
const CLAMP = { x: 140, y: 36 };           // car-centre roam limit
const CS = { x: -95, y: -147 };            // console centre
const CPLAN = { x: CS.x - 42, y: CS.y - 30, w: 84, d: 60, r: 6 };
const JX = CS.x + 16, JY = CS.y + 4;       // joystick
const STEP = 10;                            // degrees per pose-order sample

const P = k.fitProjection([
  ...k.boxCorners(FLOOR, -20, 0),
  [CPLAN.x, CPLAN.y, 33], [CPLAN.x + CPLAN.w, CPLAN.y + CPLAN.d, 33],
  [JX, JY, 34],
  [0, 0, 92],
], WIDTH, HEIGHT, { pad: 26, azimuth: -45, elevation: 32 });

const CAR = makeCar(k, G, T, P);
const CB = makeConsole(k, G, P, { CS, JX, JY, WMAX: M.WMAX });
const at = (p) => k.iso(p, P);
const S = (paths, style) => k.solidSvg(paths, style);
const Ln = (d, style) => k.lineSvg(d, style);
const D = (pts, style) => k.dotsSvg(pts, style);
const byDepth = (pts) => [...pts].sort((a, b) => k.depthOf([a[0], a[1], 0], P) - k.depthOf([b[0], b[1], 0], P));

// ---------------------------------------------------------------- floor ----
function floorSolids() {
  const out = [`<path class="iso-halo" d="${k.haloOf(FLOOR, -26, 14, P)}"/>`];
  for (const [x, y] of byDepth(k.corners(FLOOR, 26))) {
    out.push(S(k.cylinder(x, y, 8, -20, 1.6, P, 20), { tone: "lo" }));
    out.push(S(k.cylinder(x, y, 6.2, -18.4, 4.4, P, 20), { tone: "mid" }));
  }
  out.push(S(k.slabOf(FLOOR, -14, 14, P, 10, 1.2), { tone: "lo" }));
  return out.join("");
}
function floorLines() {
  const faint = [], lo = [];
  for (let x = -260; x <= 260; x += 20) (x % 100 ? faint : lo).push(k.lineOnTop([x, -180], [x, 180], 0, P));
  for (let y = -170; y <= 170; y += 20) (y % 100 ? faint : lo).push(k.lineOnTop([-270, y], [270, y], 0, P));
  const out = [Ln(faint.join(""), { tone: "faint" }), Ln(lo.join(""), { tone: "lo" })];
  out.push(Ln(k.planOutline(BOUND, 0, P, 12), { tone: "lo" }));
  const strip = { x: 260, y: -120, w: 8, d: 240, r: 1 };
  out.push(Ln(k.planOutline(k.insetPlan(strip, 0.8), 0.6, P, 3), { tone: "lo" }));
  const ticks = k.topTicks(strip.y + 4, strip.y + strip.d - 4, 10, 5, strip.x + 0.8, 0.6, [2, 3.6], P, "y");
  out.push(Ln(ticks.minor, { tone: "lo" }), Ln(ticks.major, { tone: "mid" }));
  out.push(k.faceTextSvg(k.topMatrix([strip.x + 4, 44], 0.6, P, "y"), "100", { size: 2.6, tone: "lo", anchor: "middle" }));
  return out.join("");
}

// -------------------------------------------------------------- console ----
function consoleSolids() {
  const out = [`<path class="iso-halo" d="${k.haloOf(k.insetPlan(CPLAN, -1.5), 0, 2.5, P)}" style="filter:blur(3px)"/>`];
  for (const [x, y] of byDepth(k.corners(CPLAN, 10))) out.push(S(k.cylinder(x, y, 3, 0, 2.5, P, 14), { tone: "lo" }));
  out.push(S(k.slabOf(CPLAN, 2.5, 11.5, P, 6, 0.8), { tone: "lo" }));
  out.push(S(k.slabOf(k.insetPlan(CPLAN, 2.5), 14, 1.5, P, 6, 0.4), { tone: "mid" }));
  for (let i = 0; i < 3; i++) out.push(Ln(k.planOutline({ x: CS.x - 40 + i * 9, y: CS.y - 27, w: 5.5, d: 2.6, r: 1.3 }, 15.5, P, 3), { tone: "lo" }));
  const ax = CS.x - 36, ay = CS.y - 24;
  out.push(S(k.cylinder(ax, ay, 0.8, 15.5, 14.5, P, 10), { tone: "mid" }));
  out.push(S(G.sphereOf([ax, ay, 30.8], 1.5, P, { steps: 18 }), { tone: "mid" }));
  const gx = CS.x + 37, gy = CS.y - 24;
  out.push(S(k.cylinder(gx, gy, 2.4, 15.5, 1.6, P, 14), { tone: "mid" }));
  out.push(S(k.cylinder(gx + 1.1, gy, 0.8, 17.1, 2.8, P, 10), { tone: "mid" }));
  out.push(S(G.sphereOf([gx + 1.1, gy, 20.2], 1.1, P, { steps: 14 }), { tone: "mid" }));
  out.push(S(G.solidOf([[0, 7.5], [0.9, 7.5], [3.2, 3.2, 1], [5.8, 2.5, 1], [6.6, 2.4]], G.frameAlong([JX, JY, 15.5], [0, 0, 1]), P, { steps: 22 }), { tone: "lo" }));
  out.push(S(k.cylinder(JX, JY, 9.4, 15.5, 1.2, P, 26), { tone: "mid" }));
  out.push(S(k.cylinder(JX, JY, 7.8, 16.7, 3.4, P, 30), { tone: "mid" }));
  out.push(Ln(k.knurl(JX, JY, 7.8, 16.9, 19.9, 44, P), { tone: "lo" }));
  out.push(`<g data-part="ringptr">${CB.ringPtr(0)}</g>`);
  out.push(`<g data-part="stick">${CB.stick(0, 0)}</g>`);
  const names = ["fwd", "strafe", "spin", "stop"], chars = ["前", "横", "旋", "停"];
  names.forEach((n, i) => {
    const bx = CS.x - 31 + i * 12.5;
    out.push(`<g data-button="${n}">${CB.button(bx, CS.y + 14, 0)}</g>`);
    out.push(k.faceTextSvg(k.topMatrix([bx, CS.y + 22.5], 15.5, P, "x"), chars[i], { size: 3.2, tone: "lo", anchor: "middle" }));
  });
  for (let i = 0; i < 4; i++) {
    const x = CS.x + 24 + i * 4.6;
    out.push(Ln(k.planOutline({ x, y: CS.y - 19, w: 3.4, d: 28, r: 1.6 }, 15.5, P, 3), { tone: "lo" }));
    out.push(Ln(k.lineOnTop([x + 0.5, CS.y - 5], [x + 2.9, CS.y - 5], 15.55, P), { tone: "lo" }));
  }
  out.push(`<g data-part="bars">${CB.bars([0, 0, 0, 0, 0])}</g>`);
  out.push(k.faceTextSvg(k.topMatrix([JX - 11, JY - 12.5], 15.5, P, "x"), "v", { size: 2.6, tone: "lo", anchor: "middle" }));
  out.push(k.faceTextSvg(k.topMatrix([JX + 11, JY - 12.5], 15.5, P, "x"), "w", { size: 2.6, tone: "lo", anchor: "middle" }));
  const eq = ["v1=vx+vy-75w", "v2=vx-vy-75w", "v3=vx+vy+75w", "v4=vx-vy+75w"];
  eq.forEach((line, i) => {
    const col = i % 2, row = Math.floor(i / 2);
    out.push(k.faceTextSvg(CB.frontMatrix(CS.x - 38 + col * 41, CS.y - 30.15, 11.6 - row * 3.4), line, { size: 2.3, tone: "lo", anchor: "start" }));
  });
  return out.join("");
}
function consoleShapes() {
  return [
    A.slab(CPLAN, 2.5, 11.5, { name: "console.base" }),
    A.slab(k.insetPlan(CPLAN, 2.5), 14, 1.5, { name: "console.top" }),
    A.cylinder(JX, JY, 9.4, 15.5, 1.2, { name: "console.ringseat" }),
    A.cylinder(JX, JY, 7.8, 16.7, 3.4, { name: "console.ring" }),
    A.cylinder(JX, JY, 1.6, 17, 11.5, { name: "console.stick" }),
    A.ball([JX, JY, 28.5], 4.1, { name: "console.cap" }),
    A.cylinder(CS.x - 36, CS.y - 24, 0.8, 15.5, 14.5, { name: "console.antenna" }),
    A.cylinder(CS.x - 31, CS.y + 14, 3.3, 15.5, 1.5, { name: "console.buttons" }),
  ];
}

// ------------------------------------------------------------ recording ----
// Barrels of one wheel share an item, so the audit cannot see them: check
// the clearance between every pair of roller barrels numerically.
function checkBarrels() {
  const distSeg = (p, a, b) => {
    const ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    const l2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2] || 1;
    let t = ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1] + (p[2] - a[2]) * ab[2]) / l2;
    t = Math.max(0, Math.min(1, t));
    return Math.hypot(p[0] - a[0] - ab[0] * t, p[1] - a[1] - ab[1] * t, p[2] - a[2] - ab[2] * t);
  };
  let worst = 1e9, worstPair = "";
  for (const w of WHEELS) {
    const rs = [];
    for (let i = 0; i < M.N; i++) {
      const { axis, center } = CAR.rollerLocal(w, i);
      const c = [w.x + center[0], w.y + center[1], M.axleZ + center[2]];
      rs.push([
        [c[0] - axis[0] * M.barrelHalf, c[1] - axis[1] * M.barrelHalf, c[2] - axis[2] * M.barrelHalf],
        [c[0] + axis[0] * M.barrelHalf, c[1] + axis[1] * M.barrelHalf, c[2] + axis[2] * M.barrelHalf],
      ]);
    }
    for (let i = 0; i < rs.length; i++) for (let j = i + 1; j < rs.length; j++) {
      let best = 1e9;
      for (const [from, to] of [[rs[i], rs[j]], [rs[j], rs[i]]])
        for (let t = 0; t <= 10; t++) {
          const f = t / 10;
          const p = [0, 1, 2].map((d) => from[0][d] + (from[1][d] - from[0][d]) * f);
          best = Math.min(best, distSeg(p, to[0], to[1]));
        }
      const gap = best - 2 * M.rRoller;
      if (gap < worst) { worst = gap; worstPair = `wheel${w.i} rollers ${i}/${j}`; }
    }
  }
  console.log(`barrel clearance ${worst.toFixed(2)} (${worstPair})`);
  if (worst < -1.5) { console.error("barrels intersect deeply"); process.exitCode = 1; }
}
checkBarrels();

function recordItems(R, items) {
  for (const it of items) {
    if (it.route) {
      const runs = R.route(it.name, it.route, it.r, { piece: it.name, limp: true });
      R.put({ name: it.name, at: it.at, svg: it.svg, shapes: runs });
    } else {
      R.put({
        name: it.name, at: it.at, svg: it.svg,
        shapes: (it.shapes || []).map((s, i) => {
          const sh = R.solid(s(A));
          sh.name = sh.name ?? `${it.name}.${i + 1}`;
          sh.owner = sh.owner ?? it.name;
          return sh;
        }),
      });
    }
  }
}

CAR.setPose(0, 0, 0);
const R = A.recorder(P, {});
R.put({ name: "stand.floor", at: [0, 0, -10], svg: floorSolids(), shapes: [
  R.solid(A.slab(FLOOR, -14, 14, { name: "stand.floor.slab", ground: true })),
  ...k.corners(FLOOR, 26).map(([x, y], i) => R.solid(A.cylinder(x, y, 6.2, -18.4, 4.4, { name: `stand.floor.foot${i}` }))),
] });
R.put({
  name: "console", at: [CS.x, CS.y, 8], svg: consoleSolids(),
  shapes: [
    ...k.corners(CPLAN, 10).map(([x, y], i) => R.solid(A.cylinder(x, y, 3, 0, 2.5, { name: `console.feet${i}`, owner: "console" }))),
    ...consoleShapes().map((s) => R.solid(Object.assign(s, { owner: "console" }))),
  ],
});
recordItems(R, CAR.collectItems(WHEELS.map(() => 0), [0, 0, 0]));
A.settle(R, P);

// pose-order table for the live page: one settled order per 10° of heading
const ORDERS = [];
for (let deg = 0; deg < 360; deg += STEP) {
  const items = CAR.collectItems(WHEELS.map(() => 0), [0, 0, 0], deg * Math.PI / 180);
  const Rc = A.recorder(P, {});
  recordItems(Rc, items);
  A.settle(Rc, P);
  Rc.items.sort((a, b) => a.key - b.key);
  ORDERS.push(Rc.items.map((i) => i.name));
}
CAR.setPose(0, 0, 0);

const carItems = new Map(R.items.map((i) => [i.name, i]));
const carSvg = ORDERS[0].map((n) => `<g data-item="${n}">${carItems.get(n).svg}</g>`).join("");

const svg = k.figureSvg({
  width: WIDTH,
  height: HEIGHT,
  label:
    "麦克纳姆轮运动学试验台:一辆四轮麦克纳姆小车停在划线的试验场上,旁边是带摇杆、扭环、预设按钮和四个轮速条的控制台。车体甲板中央立着 O-XYZ 坐标系和速度矢量;轮 3 触点旁画出了触点速度的分解与 α3、β3 角。摇杆给出 (vx, vy),扭环给出 ω,四个车轮按逆运动学方程转到对应转速,小车在场上真实行驶。",
  body: [
    floorSolids(),
    floorLines(),
    consoleSolids(),
    `<g data-part="car" transform="translate(0 0)">${carSvg}</g>`,
  ],
});

const DATA = { P, ORDERS, CLAMP, CS, JX, JY, STEP };

const live = readFileSync(join(HERE, "live.js"), "utf8").replace("__DATA__", JSON.stringify(DATA));

const kitNames = ["cameraOf", "iso", "pathOf", "solidSvg", "lineSvg", "dotsSvg", "onTop", "ringOf", "circleRing", "corners", "insetPlan", "extrude", "cylinder", "faceTextSvg", "lineOnTop", "axisVector", "translateAlong"];
const exportsOf = (file) => [...readFileSync(join(KIT, file), "utf8").matchAll(/^export (?:function|const) (\w+)/gm)].map((mm) => mm[1]);
const wrap = (file, name, prelude) => {
  const src = readFileSync(join(KIT, file), "utf8")
    .replace(/^import .*$\n?/gm, "")
    .replace(/^export /gm, "");
  return `const ${name} = (() => { ${prelude} ${src} return { ${exportsOf(file).join(", ")} }; })();\n`;
};
const latheScript = wrap("lathe.mjs", "G", "const cameraOf = K.cameraOf, iso = K.iso, pathOf = K.pathOf;");
const tubeScript = wrap("tube.mjs", "T", "const iso = K.iso, pathOf = K.pathOf;");
const carScript = readFileSync(join(HERE, "car.mjs"), "utf8").replace(/^export /gm, "");
const consoleScript = readFileSync(join(HERE, "console.mjs"), "utf8").replace(/^export /gm, "");
const K_DEF = `const K = { ${kitNames.join(", ")} };\n`;

const body = k.plateHtml({
  fig: "Fig M-1",
  title: "麦克纳姆轮 · 逆运动学试验台",
  hint: "拖动摇杆 · 方向键+QE",
  readout: "v = (0, 0, 0.00) · v1..v4 = 0 · x 0 y 0 θ 0°",
  keys: [
    { mark: "red", label: "红色 = 车体坐标系与速度矢量:甲板上的 (vx, vy, ω) 与各轮触点的轮速箭头 v1–v4" },
    { mark: "raised", label: "四个麦克纳姆轮:辊轴与轮轴成 45°,左右旋成对镜像(1 后左 · 2 前左 · 3 前右 · 4 后右)" },
    { mark: "edge", label: "控制台:摇杆给出 (vx, vy),扭环给出 ω,面板上刻着逆运动学方程,右侧四条是实时轮速" },
  ],
  caption:
    "假设车体为刚体、运动限于平面、车轮与地面不打滑。逆运动学把车体速度分解为四个轮速:v1=vx+vy−(H+W)ω,v2=vx−vy−(H+W)ω,v3=vx+vy+(H+W)ω,v4=vx−vy+(H+W)ω,其中 H=36、W=39 为轴距与轮距之半,L=√(H²+W²)≈53 为轮心到 O 的距离,α3 为 X 轴到 O→轮3 连线的夹角,β3=45° 为辊轴与 X 轴的夹角。轮 3 触点旁同时画出触点速度的分解:沿 X 的红色分量是电机驱动的轮速,沿辊轴的亮色分量 v3r 是辊子自由滚动的方向——正是这对 45° 辊轴让小车能横移。小车在划线边界内的试验场上真实行驶,地面网格静止,坐标甲板随车移动。",
  body: `<style>
@media (max-width: 640px){
  .iso-plate{padding:44px 10px 64px}
  .iso-stage{overflow:hidden}
  .iso-stage .iso-svg{width:118%;margin:-4% -9% -3%}
  .iso-plate-corner{font-size:9px}
  .iso-plate-corner[data-corner="readout"]{white-space:normal;max-width:58%;line-height:1.5;bottom:26px;font-size:9.5px}
}
</style><div class="iso-stage" tabindex="0" role="group" aria-label="麦克纳姆轮运动学试验台。方向键设定前后与横移,Q、E 设定自转,空格停止,也可以拖动摇杆和扭环。">${svg}</div>`,
});

const page = k.pageHtml({
  title: "麦克纳姆轮 · 逆运动学试验台",
  theme,
  body,
  script: kitScript() + K_DEF + latheScript + tubeScript + carScript + consoleScript + live,
  width: 800,
});
const name = theme === "light" ? "mecanum-light.html" : "mecanum.html";
writeFileSync(join(HERE, name), page);
console.log("wrote", name, `${(page.length / 1024).toFixed(0)} KB`, `${R.items.length} items`, `scale ${G.scaleOf(P).toFixed(3)}`);

if (AUDIT) A.auditOrExit(R, P, {});
