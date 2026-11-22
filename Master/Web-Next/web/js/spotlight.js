/* =====================================================================
 * C50C spotlight.js — 零重排卡片光标追随聚光灯（蓝图 §5.2.1）
 * 三段式约定：
 *  1. ResizeObserver / resize / scroll(passive) 预缓存当前活跃页每张
 *     .card 的 {left, top, width, height} 到内存数组（唯一允许读布局
 *     的时机，且与指针事件解耦）；
 *  2. pointermove 只把 clientX/Y 写进两个 JS 变量，零 DOM 读写；
 *  3. 实际的相对坐标计算与 style.setProperty('--mx/--my') 由
 *     scheduler.js 的全局 rAF 节拍执行（本模块只向其注册 frame 任务）。
 * ===================================================================== */
(function () {
  'use strict';
  var C = (window.C50C = window.C50C || {});

  var rects = [];          // [{ el, x, y, w, h }]
  var mx = -1e4, my = -1e4;
  var active = null;       // 当前已置 .spot-on 的卡片
  var collectPending = 0;

  /** 重采当前活跃页的卡片矩形（唯一 getBoundingClientRect 时机） */
  function collect() {
    var page = document.querySelector('.page.active');
    rects = [];
    if (!page) return;
    var cards = page.querySelectorAll('.card');
    for (var i = 0; i < cards.length; i++) {
      var r = cards[i].getBoundingClientRect();
      if (r.width > 0 && r.height > 0) {
        rects.push({ el: cards[i], x: r.left, y: r.top, w: r.width, h: r.height });
      }
    }
  }

  /** 帧合并的失效调度（resize/scroll 可能高频触发） */
  function scheduleCollect() {
    if (collectPending) return;
    collectPending = requestAnimationFrame(function () {
      collectPending = 0;
      collect();
    });
  }

  /** rAF 节拍：纯内存计算 + 写 CSS 变量（零重排） */
  function spotlightFrame() {
    var hit = null;
    for (var i = 0; i < rects.length; i++) {
      var r = rects[i];
      if (!r.el.isConnected) continue;
      if (mx >= r.x && mx <= r.x + r.w && my >= r.y && my <= r.y + r.h) {
        hit = r;
        break;
      }
    }
    if (hit !== active) {
      if (active) active.el.classList.remove('spot-on');
      if (hit) hit.el.classList.add('spot-on');
      active = hit;
    }
    if (active) {
      active.el.style.setProperty('--mx', (mx - active.x).toFixed(1) + 'px');
      active.el.style.setProperty('--my', (my - active.y).toFixed(1) + 'px');
    }
  }

  C.Spotlight = {
    init: function () {
      if (typeof ResizeObserver !== 'undefined') {
        var ro = new ResizeObserver(scheduleCollect);
        var pages = document.querySelectorAll('.page');
        for (var i = 0; i < pages.length; i++) ro.observe(pages[i]);
      }
      window.addEventListener('resize', scheduleCollect, { passive: true });
      window.addEventListener('scroll', scheduleCollect, { passive: true });
      document.addEventListener('pointermove', function (e) {
        mx = e.clientX;
        my = e.clientY;
      }, { passive: true });
      document.addEventListener('pointerleave', function () {
        mx = -1e4;
        my = -1e4;
      });
      collect();
      C.Scheduler.frame(spotlightFrame);
    },

    /** 路由切换 / 主题切换后调用：活跃页变了，矩形全部过期 */
    refresh: function () { scheduleCollect(); },
  };
})();
