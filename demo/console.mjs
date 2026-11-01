// Console bits that both the build (static page) and the live page rebuild:
// stick, buttons, twist-ring pointer, wheel-speed bars. k/G/P passed in;
// D = { CS, JX, JY, WMAX }.

export function makeConsole(k, G, P, D) {
  const at = (p) => k.iso(p, P);
  const S = (paths, style) => k.solidSvg(paths, style);
  const Ln = (d, style) => k.lineSvg(d, style);
  const Dt = (pts, style) => k.dotsSvg(pts, style);
  const { CS, JX, JY, WMAX } = D;

  const frontMatrix = (x, yFace, z) => {
    const o = at([x, yFace, z]);
    const u = at([x - 1, yFace, z]), v = at([x, yFace, z - 1]);
    const f = (n) => Math.round(n * 1000) / 1000;
    return `matrix(${f(u[0] - o[0])} ${f(u[1] - o[1])} ${f(v[0] - o[0])} ${f(v[1] - o[1])} ${f(o[0])} ${f(o[1])})`;
  };

  function stick(ux, uy) {
    const axis = G.unit3([ux * 0.38, uy * 0.38, 1]);
    const F = G.frameAlong([JX, JY, 15.5], axis);
    return S(G.disc(0, 10.5, 1.55, F, P, { steps: 14 }), { tone: "mid" }) +
      S(G.sphereOf(G.add3([JX, JY, 15.5], G.mul3(axis, 13)), 4.1, P, { steps: 28 }), { tone: "hi", lit: true });
  }

  function button(bx, by, press) {
    const z = 17 - press;
    return S(k.cylinder(bx, by, 3.3, 15.5, z - 15.5, P, 20), { tone: "mid" }) +
      S(G.solidOf([[0, 3.3], [1.8, 3.3], [2.9, 2.5, 1], [3.4, 0.8, 1], [3.5, 0]], G.frameAlong([bx, by, z], [0, 0, 1]), P, { steps: 20 }), { tone: "hi" });
  }

  function ringPtr(omega) {
    const a = omega / WMAX * 4.712;
    const c = Math.cos(a), s = Math.sin(a);
    return Ln(k.lineOnTop([JX + c * 3.2, JY + s * 3.2], [JX + c * 7.2, JY + s * 7.2], 20.3, P), { tone: "hi" }) +
      Dt([at([JX + c * 7.2, JY + s * 7.2, 20.3])], { size: 0.6, tone: "hi" });
  }

  function bars(vi) {
    let out = "";
    for (let i = 0; i < 4; i++) {
      const x = CS.x + 24 + i * 4.6, cy = CS.y - 5;
      const v = Math.max(-630, Math.min(630, vi[i + 1] || 0));
      if (Math.abs(v) < 4) continue;
      const h = v / 630 * 12.5;
      const y0 = Math.min(cy, cy + h), y1 = Math.max(cy, cy + h);
      const pts = [[x + 0.5, y0], [x + 2.9, y0], [x + 2.9, y1], [x + 0.5, y1]]
        .map(([px, py]) => at([px, py, 15.6]).map((n) => n.toFixed(1)).join(",")).join(" ");
      out += `<polygon points="${pts}" fill="var(--anatomy-red)" stroke="none"/>`;
    }
    return out;
  }

  return { stick, button, ringPtr, bars, frontMatrix };
}
