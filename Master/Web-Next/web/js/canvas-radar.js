/* =====================================================================
 * C50C canvas-radar.js — 720 分桶极坐标持久化点云 + 移动角区高亮
 * 蓝图 §3.1：点云按 0.5° 分桶驻留、按帧龄渐隐；移动扫描角区叠加高亮。
 * 坐标经 ResizeObserver 预缓存（rect.w/h/dpr），绘制阶段零 DOM 读取。
 * ===================================================================== */
(function () {
  'use strict';
  var C = (window.C50C = window.C50C || {});
  var Store = C.Store;

  var BUCKETS = 720;        // 0.5° 分桶（蓝图 §3.1 canvas-radar.js）
  var FADE_MS = 6000;       // 点云余晖时长
  var SECTOR_DEG = 34;      // 移动角区宽度
  var SECTOR_DEG_MS = 0.085; // 角区转速（deg/ms ≈ 85°/s）
  var RANGE_MIN = 2;        // 自适应量程下限 m
  var RANGE_MAX = 15;

  /** 720 桶驻留点云：半径 + 最近命中时刻 */
  var bucketR = new Float32Array(BUCKETS);
  var bucketT = new Float64Array(BUCKETS);
  var bucketSeen = new Uint8Array(BUCKETS);

  function ingest(scan) {
    var p = scan.p, now = scan.t;
    for (var i = 0; i < p.length; i++) {
      var deg = p[i][0], r = p[i][1];
      if (!isFinite(deg) || !(r > 0)) continue;
      var idx = Math.round((((deg % 360) + 360) % 360) * 2) % BUCKETS;
      bucketR[idx] = r;
      bucketT[idx] = now;
      bucketSeen[idx] = 1;
    }
  }

  function clearBuckets() {
    bucketSeen.fill(0);
  }

  /** 解析当前生效主题（meta 为 light dark 时跟随系统） */
  function isDark() {
    var content = document.querySelector('meta[name="color-scheme"]').content;
    if (content === 'dark') return true;
    if (content === 'light') return false;
    return window.matchMedia('(prefers-color-scheme: dark)').matches;
  }

  /**
   * 创建一个雷达画布实例。
   * @param canvas 画布元素
   * @param pages  仅在这些路由页活跃时绘制
   */
  function create(canvas, pages) {
    var rect = { w: 0, h: 0, dpr: 1 };
    var ctx = canvas.getContext('2d');
    var lastIngestT = -1;
    var rangeShown = 6;
    // 用户量程覆盖：null = 跟随 radiusMax(now) 自适应；滚轮置数值，双击回 null
    var rangeOverride = null;

    if (typeof ResizeObserver !== 'undefined') {
      // 预缓存坐标（蓝图 §5.2.1）：尺寸变化才触碰布局，绘制循环零重排
      new ResizeObserver(function (entries) {
        var e = entries[entries.length - 1];
        if (!e) return;
        rect.w = e.contentRect.width;
        rect.h = e.contentRect.height;
        rect.dpr = Math.min(window.devicePixelRatio || 1, 2);
        canvas.width = Math.max(1, Math.round(rect.w * rect.dpr));
        canvas.height = Math.max(1, Math.round(rect.h * rect.dpr));
      }).observe(canvas);
    }

    function radiusMax(now) {
      var fresh = 0;
      for (var i = 0; i < BUCKETS; i++) {
        if (bucketSeen[i] && now - bucketT[i] < FADE_MS && bucketR[i] > fresh) {
          fresh = bucketR[i];
        }
      }
      return Math.min(RANGE_MAX, Math.max(RANGE_MIN, fresh * 1.06));
    }

    function draw(now) {
      var w = rect.w, h = rect.h, dpr = rect.dpr;
      if (!w || !h) return;
      var dark = isDark();
      var cx = w / 2, cy = h / 2;
      var Rpx = Math.min(cx, cy) - 16;
      if (Rpx <= 10) return;

      // 量程平滑跟随：未交互时跟随 radiusMax(now) 自适应（行为与之前一致）；
      // 用户滚轮缩放后以指针为参照放大/缩小显示量程，实时应用到绘制
      var target = rangeOverride != null
        ? Math.max(RANGE_MIN, Math.min(RANGE_MAX, rangeOverride))
        : radiusMax(now);
      rangeShown += (target - rangeShown) * 0.06;

      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, h);

      // HUD 托底圆盘（赛车游戏式）：半透明，视频画面透过可见，仅托住扫掠可读性
      ctx.beginPath();
      ctx.arc(cx, cy, Rpx + 10, 0, Math.PI * 2);
      ctx.fillStyle = dark ? 'rgba(10,12,18,0.30)' : 'rgba(238,241,246,0.34)';
      ctx.fill();

      // 距离环
      ctx.lineWidth = 1;
      ctx.strokeStyle = dark ? 'rgba(231,233,239,0.09)' : 'rgba(27,30,38,0.10)';
      ctx.beginPath();
      for (var ring = 1; ring <= 3; ring++) {
        ctx.moveTo(cx + (Rpx * ring / 3), cy);
        ctx.arc(cx, cy, Rpx * ring / 3, 0, Math.PI * 2);
      }
      ctx.stroke();

      // 移动角区高亮（径向渐变楔形）
      var sector0 = (now * SECTOR_DEG_MS) % 360;
      var a0 = sector0 * Math.PI / 180;
      var a1 = (sector0 + SECTOR_DEG) * Math.PI / 180;
      var grad = ctx.createRadialGradient(cx, cy, 0, cx, cy, Rpx);
      var acc = dark ? '139,162,255' : '76,102,245';
      grad.addColorStop(0, 'rgba(' + acc + ',0.02)');
      grad.addColorStop(0.85, 'rgba(' + acc + ',0.10)');
      grad.addColorStop(1, 'rgba(' + acc + ',0.0)');
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.arc(cx, cy, Rpx, a0, a1);
      ctx.closePath();
      ctx.fillStyle = grad;
      ctx.fill();
      ctx.beginPath();
      ctx.moveTo(cx + Math.cos(a1) * Rpx * 0.2, cy + Math.sin(a1) * Rpx * 0.2);
      ctx.lineTo(cx + Math.cos(a1) * Rpx, cy + Math.sin(a1) * Rpx);
      ctx.strokeStyle = 'rgba(' + acc + ',0.45)';
      ctx.stroke();

      // 点云：deg 0 = 车头方向（画面上方），逆时针为正 → 屏幕 y 取负
      for (var i = 0; i < BUCKETS; i++) {
        if (!bucketSeen[i]) continue;
        var age = now - bucketT[i];
        if (age >= FADE_MS) continue;
        var alpha = Math.max(0.12, 1 - age / FADE_MS);
        var r = bucketR[i];
        if (r > rangeShown) continue;
        var ang = (i / 2) * Math.PI / 180;
        var px = cx + Math.sin(ang) * (r / rangeShown) * Rpx;
        var py = cy - Math.cos(ang) * (r / rangeShown) * Rpx;
        ctx.fillStyle = dark
          ? 'rgba(120,200,255,' + alpha.toFixed(3) + ')'
          : 'rgba(30,90,180,' + alpha.toFixed(3) + ')';
        ctx.fillRect(px - 1.1, py - 1.1, 2.2, 2.2);
      }

      // 车体
      ctx.fillStyle = dark ? '#e7e9ef' : '#1b1e26';
      ctx.beginPath();
      ctx.moveTo(cx, cy - 7);
      ctx.lineTo(cx + 5, cy + 6);
      ctx.lineTo(cx - 5, cy + 6);
      ctx.closePath();
      ctx.fill();

      // 量程标注
      ctx.fillStyle = dark ? 'rgba(231,233,239,0.45)' : 'rgba(27,30,38,0.5)';
      ctx.font = '10px ui-monospace, monospace';
      ctx.textAlign = 'right';
      ctx.fillText(rangeShown.toFixed(1) + ' m', w - 8, h - 8);
    }

    function frame(now, flags) {
      if (pages.indexOf(Store.state.ui.page) === -1) return;
      if (!rect.w || !rect.h) return;
      var scan = Store.state.scan;
      if (scan && scan.t !== lastIngestT) {
        ingest(scan);
        lastIngestT = scan.t;
      }
      draw(now);
    }

    // 滚轮缩放（用户量程覆盖系数）：上滚拉近（量程缩小）、下滚拉远；
    // 以当前显示量程 rangeShown 为基准逐级缩放。双击复位回 radiusMax(now)
    // 自适应——只有显式交互才动 rangeOverride，未交互时行为与之前一致。
    canvas.addEventListener('wheel', function (e) {
      e.preventDefault();
      e.stopPropagation();
      var base = rangeOverride != null ? rangeOverride : rangeShown;
      var factor = e.deltaY < 0 ? 1 / 1.18 : 1.18;
      rangeOverride = Math.max(RANGE_MIN, Math.min(RANGE_MAX, base * factor));
    }, { passive: false });
    canvas.addEventListener('dblclick', function (e) {
      e.preventDefault();
      e.stopPropagation();
      rangeOverride = null;   // 回自适应：下一帧平滑跟回 radiusMax(now)
    });

    return { frame: frame };
  }

  C.Radar = { create: create, ingest: ingest, clear: clearBuckets, BUCKETS: BUCKETS };
})();
