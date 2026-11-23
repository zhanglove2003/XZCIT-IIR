/* =====================================================================
 * C50C scheduler.js — 全应用唯一的 requestAnimationFrame 渲染循环
 * 蓝图 §5.2.4：渲染器在每个 rAF 节拍依据 take() 的脏位批量更新
 * 变化了的文本节点（先比对 el._lastText 再写 textContent），并按需
 * 重绘当前可见的 Canvas。严禁任何其他地方再起 rAF 循环。
 * ===================================================================== */
(function () {
  'use strict';
  var C = (window.C50C = window.C50C || {});

  /** 按脏位触发的 DOM 文本渲染器：{ flag, fn(now, flags) } */
  var flaggedTasks = [];
  /** 每帧都跑的低成本任务：聚光灯 / 按可见性节流的 Canvas 渲染 */
  var frameTasks = [];
  var running = false;
  var ticked = false;       // 首个 rAF 节拍是否已到来
  var fallbackTimer = 0;    // rAF 完全停摆时的 setTimeout 泵帧句柄

  function tick(now) {
    ticked = true;
    var flags = C.Store.take();
    var i;
    for (i = 0; i < frameTasks.length; i++) {
      try { frameTasks[i](now, flags); } catch (e) { console.error('[raf] frame task', e); }
    }
    for (i = 0; i < flaggedTasks.length; i++) {
      if (flags & flaggedTasks[i].flag) {
        try { flaggedTasks[i].fn(now, flags); } catch (e) { console.error('[raf] task', e); }
      }
    }
    // 泵帧兜底接管后，落回来的真实 rAF 回调直接退出，避免双循环
    if (!fallbackTimer) requestAnimationFrame(tick);
  }

  /** 内嵌 webview / 隐藏页签里 rAF 可能被完全暂停：以 ~60fps 泵帧续命 */
  function pumpFallback() {
    tick(performance.now());
    fallbackTimer = setTimeout(pumpFallback, 16);
  }

  C.Scheduler = {
    /** 注册按脏位触发的渲染器（DOM 文本更新全部走这里） */
    task: function (flag, fn) { flaggedTasks.push({ flag: flag, fn: fn }); },

    /** 注册每帧任务（聚光灯、可见画布的重绘节流在此自行判断） */
    frame: function (fn) { frameTasks.push(fn); },

    /** 启动唯一渲染循环（幂等） */
    start: function () {
      if (running) return;
      running = true;
      ticked = false;
      requestAnimationFrame(tick);
      // 真实浏览器前台 1.5s 内必有节拍；始终无节拍说明 rAF 被宿主暂停，
      // 切 setTimeout 泵帧保证 HUD/画布/许可显示不冻结。
      setTimeout(function () {
        if (!ticked && !fallbackTimer) pumpFallback();
      }, 1500);
    },
  };
})();
