/* =====================================================================
 * C50C canvas-map.js — 占据栅格 Offscreen 纹理缓存 + 缩放拖拽 + 多航点闭环
 * 蓝图 §3.1/§5.2.4：
 *  - 栅格 {t:"map"} 的 zlib+base64 数据仅在 map.seq（或主题）变化时解码
 *    一次并绘入 OffscreenCanvas 纹理；5Hz 位姿只重绘上层机器人箭头与
 *    航点路径（drawImage 一张缓存底图 + 矢量叠加，不重建 ImageData）；
 *  - 滚轮以光标为锚缩放、拖拽平移，全部只改视图内存并置 needsDraw，
 *    由 scheduler 统一在 rAF 里重绘。
 * 栅格行约定（ROS OccupancyGrid）：第 0 行是 y 最小（底边）行，建纹理时
 * 须垂直翻行；cell 值 0=空闲 100=占据 255(-1)=未知。
 * ===================================================================== */
(function () {
  'use strict';
  var C = (window.C50C = window.C50C || {});
  var Store = C.Store;

  var SCALE_MIN = 3;     // px / m 下限
  var SCALE_MAX = 480;
  // 显示旋转（对齐旧 console_page.html 方向约定）：车头 +x（0°）朝正西（画面正左），
  // 车左 +y（连续墙）在画面正南（底边）；整图、位姿箭头、轨迹、航点与点击反投影共用此刚体旋转。
  var MAP_ROT = Math.PI;
  var COS_R = Math.cos(MAP_ROT);
  var SIN_R = Math.sin(MAP_ROT);

  var canvas = null;
  var ctx = null;
  var rect = { w: 0, h: 0, dpr: 1 };

  var view = { cx: 0, cy: 0, scale: 40 };      // 相机中心（世界 m）与比例
  var tex = null;                              // OffscreenCanvas / 离屏 canvas
  var texSeq = -1;
  var texTheme = '';
  var texGeom = null;                          // {w,h,res,ox,oy}
  var decoding = false;
  var decodeFailed = false;
  var needsDraw = false;
  var firstFitDone = false;

  var drag = null;                             // {sx,sy,cx,cy,moved}
  var trail = [];                              // 位姿轨迹（最新在后）

  function isDark() {
    var content = document.querySelector('meta[name="color-scheme"]').content;
    if (content === 'dark') return true;
    if (content === 'light') return false;
    return window.matchMedia('(prefers-color-scheme: dark)').matches;
  }

  function themeKey() { return isDark() ? 'dark' : 'light'; }

  /* ---------------- 世界 ⇄ 屏幕（CSS 像素） ---------------- */

  function project(wx, wy) {
    var dx = wx - view.cx;
    var dy = wy - view.cy;
    return {
      x: rect.w / 2 + (dx * COS_R - dy * SIN_R) * view.scale,
      y: rect.h / 2 - (dx * SIN_R + dy * COS_R) * view.scale,
    };
  }

  function unproject(sx, sy) {
    var u = (sx - rect.w / 2) / view.scale;
    var v = -(sy - rect.h / 2) / view.scale;
    return {
      x: view.cx + u * COS_R + v * SIN_R,
      y: view.cy - u * SIN_R + v * COS_R,
    };
  }

  /* ---------------- 栅格解码与纹理重建 ---------------- */

  function ensureTexture() {
    var m = Store.state.map;
    if (!m || !m.d || !m.w || !m.h) return;
    var theme = themeKey();
    if (tex && texSeq === m.seq && texTheme === theme) return;
    if (decoding || decodeFailed) return;
    decoding = true;

    var bin;
    try { bin = atob(m.d); } catch (e) { decoding = false; decodeFailed = true; return; }
    var bytes = new Uint8Array(bin.length);
    for (var i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);

    if (typeof DecompressionStream === 'undefined') {
      decoding = false;
      decodeFailed = true;
      console.error('[map] 浏览器缺少 DecompressionStream，无法解压栅格');
      return;
    }

    new Response(new Blob([bytes]).stream().pipeThrough(new DecompressionStream('deflate')))
      .arrayBuffer()
      .then(function (buf) {
        decoding = false;
        var cur = Store.state.map;
        if (!cur || cur.seq !== m.seq) return;   // 已有更新帧，等下一轮
        buildTexture(cur, new Uint8Array(buf), theme);
      })
      .catch(function (e) {
        decoding = false;
        decodeFailed = true;
        console.error('[map] 栅格解压失败', e);
      });
  }

  function buildTexture(m, data, theme) {
    if (data.length < m.w * m.h) return;
    var makeCanvas = typeof OffscreenCanvas !== 'undefined'
      ? function (w, h) { return new OffscreenCanvas(w, h); }
      : function (w, h) {
          var c = document.createElement('canvas');
          c.width = w; c.height = h;
          return c;
        };
    var off = makeCanvas(m.w, m.h);
    var octx = off.getContext('2d');
    var img = octx.createImageData(m.w, m.h);
    var px = img.data;

    var occ = theme === 'dark' ? [221, 226, 236] : [38, 43, 54];   // 占据
    var free = theme === 'dark' ? [24, 28, 36] : [231, 235, 241];  // 空闲
    var unk = theme === 'dark' ? [13, 14, 18] : [180, 186, 198];   // 未知
    for (var iy = 0; iy < m.h; iy++) {
      var srcRow = iy * m.w;
      var dstRow = (m.h - 1 - iy) * m.w;
      for (var ix = 0; ix < m.w; ix++) {
        var v = data[srcRow + ix];
        var c = v === 255 ? unk : (v >= 65 ? occ : (v <= 25 ? free : unk));
        var o = (dstRow + ix) * 4;
        px[o] = c[0]; px[o + 1] = c[1]; px[o + 2] = c[2]; px[o + 3] = 255;
      }
    }
    octx.putImageData(img, 0, 0);
    tex = off;
    texSeq = m.seq;
    texTheme = theme;
    texGeom = { w: m.w, h: m.h, res: m.res, ox: m.ox, oy: m.oy };
    trail.length = 0;
    if (!firstFitDone) fit();
    needsDraw = true;
  }

  /* ---------------- 视图操作（fit / zoom / pan） ---------------- */

  function fit() {
    var g = texGeom || Store.state.map;
    if (!g || !g.w) return;
    var worldW = g.w * g.res, worldH = g.h * g.res;
    view.cx = g.ox + worldW / 2;
    view.cy = g.oy + worldH / 2;
    if (rect.w > 4 && rect.h > 4) {
      view.scale = Math.min(rect.w / (worldW * 1.12), rect.h / (worldH * 1.12));
      view.scale = Math.max(SCALE_MIN, Math.min(SCALE_MAX, view.scale));
      firstFitDone = true;
    }
    needsDraw = true;
  }

  /** 以屏幕点 (sx,sy) 为锚缩放 f 倍：锚点世界坐标保持不动 */
  function zoomAt(sx, sy, f) {
    var before = unproject(sx, sy);
    view.scale = Math.max(SCALE_MIN, Math.min(SCALE_MAX, view.scale * f));
    var after = unproject(sx, sy);
    view.cx += before.x - after.x;
    view.cy += before.y - after.y;
    needsDraw = true;
  }

  /* ---------------- 绘制 ---------------- */

  function strokeLine(pts, color, width, dash, close) {
    if (pts.length < 2) return;
    ctx.save();
    if (dash) ctx.setLineDash(dash);
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.beginPath();
    for (var i = 0; i < pts.length; i++) {
      var p = project(pts[i][0], pts[i][1]);
      if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
    }
    if (close) ctx.closePath();
    ctx.stroke();
    ctx.restore();
  }

  function drawDot(wx, wy, color, r) {
    var p = project(wx, wy);
    ctx.beginPath();
    ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
  }

  function drawLabel(wx, wy, text, color) {
    var p = project(wx, wy);
    ctx.font = '600 11px ui-monospace, monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = color;
    ctx.fillText(text, p.x, p.y);
  }

  function draw(now) {
    var w = rect.w, h = rect.h, dpr = rect.dpr;
    if (!w || !h) return;
    var dark = isDark();
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = dark ? '#101218' : '#eef1f5';
    ctx.fillRect(0, 0, w, h);

    // 缓存底图（Offscreen 纹理）：按 MAP_ROT 旋转后绘制（一次 drawImage）
    if (tex && texGeom) {
      var g = texGeom;
      var tl = project(g.ox, g.oy + g.h * g.res);          // 世界左上（y 大）
      ctx.save();
      ctx.translate(tl.x, tl.y);
      ctx.rotate(-MAP_ROT);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(tex, 0, 0, g.w * g.res * view.scale, g.h * g.res * view.scale);
      ctx.restore();
    } else {
      var waiting = decodeFailed ? '栅格解压失败' : '等待地图…';
      ctx.fillStyle = dark ? 'rgba(231,233,239,0.4)' : 'rgba(27,30,38,0.45)';
      ctx.font = '12px ui-monospace, monospace';
      ctx.textAlign = 'center';
      ctx.fillText(waiting, w / 2, h / 2 + 4);
      // 等待态不能让人干瞪眼：把最可能的排查入口写在这儿（垫片一停，
      // /map 就永远不来，这与 AMCL 无关）
      ctx.font = '11px ui-monospace, monospace';
      ctx.globalAlpha = 0.72;
      ctx.fillText(
        decodeFailed ? '刷新页面重试；仍失败请重启 c50c-web-next' : '若长时间无图，检查垫片服务 c50c-web-bridge',
        w / 2,
        h / 2 + 24,
      );
      ctx.globalAlpha = 1;

      // 提示文字先画、箭头后画：箭头压在（可能更长的）首行文字中间会糊成一团。
      // 有历史位姿时把箭头画在文案上方一档。
      var wp = Store.poseUsable() ? Store.state.pose : null;
      if (wp) {
        var wy = h / 2 - 26;
        ctx.save();
        ctx.translate(w / 2, wy);
        ctx.rotate(-(wp.th + MAP_ROT));
        ctx.fillStyle = dark ? 'rgba(255,141,61,0.85)' : 'rgba(217,107,16,0.85)';
        ctx.beginPath();
        ctx.moveTo(11, 0);
        ctx.lineTo(-7, 6.5);
        ctx.lineTo(-3.5, 0);
        ctx.lineTo(-7, -6.5);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
      }
    }

    var status = Store.state.status;
    var patrol = status && status.patrol;

    // 已保存巡航路线：闭环实线 + 当前目标高亮
    if (patrol && Array.isArray(patrol.route) && patrol.route.length > 1) {
      strokeLine(patrol.route, dark ? 'rgba(139,162,255,0.85)' : 'rgba(76,102,245,0.8)', 2, null, true);
      for (var i = 0; i < patrol.route.length; i++) {
        var active = patrol.status === 'running' && i === (patrol.index || 0);
        drawDot(patrol.route[i][0], patrol.route[i][1],
          active ? (dark ? '#f0b04a' : '#b06f08') : (dark ? '#8ba2ff' : '#4c66f5'),
          active ? 7 : 5);
        drawLabel(patrol.route[i][0], patrol.route[i][1], String(i + 1), dark ? '#0d0e12' : '#ffffff');
      }
    }

    // 航点草稿：虚线闭环 + 序号
    var draft = Store.state.ui.draft;
    if (draft.length) {
      strokeLine(draft, dark ? 'rgba(62,207,142,0.9)' : 'rgba(15,138,99,0.85)', 1.6, [6, 5], draft.length > 2);
      for (var j = 0; j < draft.length; j++) {
        drawDot(draft[j].x, draft[j].y, dark ? '#3ecf8e' : '#0f8a63', 6);
        drawLabel(draft[j].x, draft[j].y, String(j + 1), dark ? '#0d0e12' : '#ffffff');
      }
    }

    // 单点导航目标（来自 status.nav.goal）
    var nav = status && status.nav;
    if (nav && nav.goal) {
      var gx = nav.goal.x, gy = nav.goal.y;
      var p = project(gx, gy);
      ctx.strokeStyle = dark ? '#ff5d6e' : '#cc3344';
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      ctx.moveTo(p.x - 9, p.y); ctx.lineTo(p.x + 9, p.y);
      ctx.moveTo(p.x, p.y - 9); ctx.lineTo(p.x, p.y + 9);
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(p.x, p.y, 13, 0, Math.PI * 2);
      ctx.stroke();
    }

    // 位姿轨迹与机器人箭头仅在底图纹理就绪时按世界坐标绘制；
    // 无图等待态已在上方（文字上档）单绘过箭头，不再按默认原点叠画压字。
    if (tex && texGeom && trail.length > 1) {
      strokeLine(trail, dark ? 'rgba(240,176,74,0.5)' : 'rgba(176,111,8,0.45)', 1.4);
    }

    // 机器人箭头：x 前 y 左，th 逆时针，叠合 MAP_ROT 显示旋转 → 屏幕 y 取负
    // 位姿过期就不画：把两秒前的位置画成当前，比不画更误导。
    var pose = Store.poseUsable() ? Store.state.pose : null;
    if (tex && texGeom && pose) {
      var pr = project(pose.x, pose.y);
      ctx.save();
      ctx.translate(pr.x, pr.y);
      ctx.rotate(-(pose.th + MAP_ROT));
      ctx.fillStyle = dark ? '#ff8d3d' : '#d96b10';
      ctx.beginPath();
      ctx.moveTo(11, 0);
      ctx.lineTo(-7, 6.5);
      ctx.lineTo(-3.5, 0);
      ctx.lineTo(-7, -6.5);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
    }

    // 比例尺
    var barM = view.scale > 90 ? 0.5 : (view.scale > 30 ? 1 : 5);
    var bx = 14, by = h - 14;
    ctx.strokeStyle = dark ? 'rgba(231,233,239,0.6)' : 'rgba(27,30,38,0.55)';
    ctx.fillStyle = ctx.strokeStyle;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(bx, by - 4); ctx.lineTo(bx, by);
    ctx.lineTo(bx + barM * view.scale, by);
    ctx.lineTo(bx + barM * view.scale, by - 4);
    ctx.stroke();
    ctx.font = '10px ui-monospace, monospace';
    ctx.textAlign = 'left';
    ctx.fillText(barM + ' m', bx + 2, by - 7);
  }

  /* ---------------- 指针交互（拖拽 / 点选 / 缩放） ---------------- */

  function bindPointer() {
    canvas.addEventListener('pointerdown', function (e) {
      if (e.button !== 0) return;
      try { canvas.setPointerCapture(e.pointerId); } catch (err) { /* 指针已失效时忽略 */ }
      var r = canvas.getBoundingClientRect();   // 交互事件里读布局是允许的
      drag = { sx: e.clientX - r.left, sy: e.clientY - r.top, cx: view.cx, cy: view.cy, moved: false };
      canvas.classList.add('grabbing');
    });

    canvas.addEventListener('pointermove', function (e) {
      if (!drag) return;
      var r = canvas.getBoundingClientRect();
      var sx = e.clientX - r.left, sy = e.clientY - r.top;
      var dx = sx - drag.sx, dy = sy - drag.sy;
      if (Math.abs(dx) + Math.abs(dy) > 4) drag.moved = true;
      if (!drag.moved) return;
      var du = dx / view.scale, dv = -dy / view.scale;
      view.cx = drag.cx - (du * COS_R + dv * SIN_R);
      view.cy = drag.cy - (-du * SIN_R + dv * COS_R);
      needsDraw = true;
    });

    function endDrag(e) {
      if (!drag) return;
      var wasClick = !drag.moved;
      drag = null;
      canvas.classList.remove('grabbing');
      if (!wasClick) return;
      var r = canvas.getBoundingClientRect();
      var wpt = unproject(e.clientX - r.left, e.clientY - r.top);
      var g = texGeom;
      if (!g) {
        // 没栅格时世界坐标无从谈起：明确告诉用户「现在点不了」，
        // 而不是静默吞掉这次点击（用户只会以为界面坏了）。
        if (Store.state.ui.mapMode !== 'pan') {
          Store.emit('toast', { message: '地图还没加载，无法选点' });
        }
        return;
      }
      if (Store.state.ui.mapMode === 'nav') {
        Store.emit('nav_click', { x: wpt.x, y: wpt.y });
      } else if (Store.state.ui.mapMode === 'way') {
        Store.emit('way_click', { x: wpt.x, y: wpt.y });
      }
    }

    canvas.addEventListener('pointerup', endDrag);
    canvas.addEventListener('pointercancel', function () { drag = null; canvas.classList.remove('grabbing'); });

    canvas.addEventListener('wheel', function (e) {
      e.preventDefault();
      var r = canvas.getBoundingClientRect();
      var f = e.deltaY < 0 ? 1.13 : 1 / 1.13;
      zoomAt(e.clientX - r.left, e.clientY - r.top, f);
    }, { passive: false });
  }

  /* ---------------- 初始化与调度接口 ---------------- */

  function init(el) {
    canvas = el;
    ctx = canvas.getContext('2d');
    if (typeof ResizeObserver !== 'undefined') {
      new ResizeObserver(function (entries) {
        var e = entries[entries.length - 1];
        if (!e) return;
        rect.w = e.contentRect.width;
        rect.h = e.contentRect.height;
        rect.dpr = Math.min(window.devicePixelRatio || 1, 2);
        canvas.width = Math.max(1, Math.round(rect.w * rect.dpr));
        canvas.height = Math.max(1, Math.round(rect.h * rect.dpr));
        if (!firstFitDone) fit();
        needsDraw = true;
      }).observe(canvas);
    }
    bindPointer();
  }

  C.MapView = {
    init: init,
    fit: fit,
    /** 栅格是否已解码到位（app.js 用它决定选点按钮可用性） */
    ready: function () { return !!texGeom; },
    /** 工具栏缩放：中心锚点 */
    zoom: function (f) { zoomAt(rect.w / 2, rect.h / 2, f); },
    invalidate: function () { needsDraw = true; },

    /** scheduler.frame 注册的主绘制入口（蓝图：按需重绘可见画布） */
    frame: function (now, flags) {
      if (!canvas) return;
      var F = Store.FLAGS;
      // 地图已并入控制页第三 tab：仅在控制页渲染
      if (Store.state.ui.page === 'control') {
        if (flags & F.MAPTEX) ensureTexture();
        if (flags & F.POSE) {
          var pose = Store.poseUsable() ? Store.state.pose : null;
          if (pose) {
            trail.push([pose.x, pose.y]);
            if (trail.length > 400) trail.shift();
          }
          needsDraw = true;
        }
        if (flags & F.ROUTE) needsDraw = true;
        if (flags & F.THEME) { texTheme = ''; ensureTexture(); needsDraw = true; }
        if (needsDraw) { draw(now); needsDraw = false; }
      } else {
        needsDraw = true;   // 回到控制页时强制重绘一帧
      }
    },
  };
})();
