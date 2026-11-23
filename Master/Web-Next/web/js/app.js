/* =====================================================================
 * C50C app.js — 入口：哈希路由、主题切换、控件绑定、DOM 渲染器
 * 蓝图 §5：
 *  - 4 页路由 #/overview #/control #/map #/logs（扁平一级导航）；
 *  - 非活跃页 content-visibility:hidden + inert（layout.css + 此处切换）；
 *  - 主题切换只改 meta[name=color-scheme]，startViewTransition 包裹；
 *  - DOM 文本更新全部经 Scheduler.task(脏位) 在唯一 rAF 里执行，
 *    先比对 el._lastText 再写 textContent（§5.2.4）。
 * ===================================================================== */
(function () {
  'use strict';
  var C = (window.C50C = window.C50C || {});
  var Store = C.Store;
  var F = Store.FLAGS;
  var Scheduler = C.Scheduler;
  var ui = Store.state.ui;

  function $(id) { return document.getElementById(id); }

  /* ================= 通用工具 ================= */

  /** 音量条填充层同步：--spk-ratio（0..1）驱动 #spk-vol-fill 的加粗活动段。
   *  变量同时写到 input 与外层 box——填充层是 box 的子元素，只认 box 上的值。 */
  function syncSpkFill(el) {
    var min = Number(el.min) || 0;
    var max = Number(el.max) || 100;
    var ratio = max > min ? (Number(el.value) - min) / (max - min) : 0;
    var s = String(ratio);
    el.style.setProperty('--spk-ratio', s);
    var box = el.parentElement;
    if (box) box.style.setProperty('--spk-ratio', s);
  }

  /** 喇叭音量的本地待定值：状态帧里的 audio.volume 来自板端 5 s 一轮的
   *  设备快照，刚调完音量时还是旧值，不能让它把界面打回去。
   *  { v: 想要的值, t: 时间, sent: 是否已发命令 }——音量回执定值后
   *  sent 清掉；状态帧追上 v 或 8 s 超时即整个清除，恢复常规回写。 */
  var spkPend = null;

  /** 麦克风录音/播放状态（与服务端 status.audio.mic 及 mic_status 回执同步）。 */
  var micUi = {
    phase: 'idle',
    recording: false,
    playing: false,
    has_recording: false,
    duration_s: null,
    elapsed_s: 0,
    pending: null,
    pendingAt: 0,
  };

  function applyMicSnapshot(m) {
    if (!m || typeof m !== 'object') return;
    micUi.phase = typeof m.phase === 'string' ? m.phase : 'idle';
    micUi.recording = !!m.recording;
    micUi.playing = !!m.playing;
    micUi.has_recording = !!m.has_recording;
    micUi.duration_s = typeof m.duration_s === 'number' ? m.duration_s : null;
    micUi.elapsed_s = typeof m.elapsed_s === 'number' ? m.elapsed_s : 0;
    if (micUi.pending) {
      var reached =
        (micUi.pending === 'start' && (micUi.recording || micUi.phase === 'recording')) ||
        (micUi.pending === 'stop' && micUi.phase === 'idle' && !micUi.recording) ||
        (micUi.pending === 'play' && (micUi.playing || micUi.phase === 'playing'));
      if (reached || Date.now() - micUi.pendingAt > 10000) {
        micUi.pending = null;
      }
    }
  }

  function renderMic(audio) {
    var recEl = $('sn-mic-rec');
    var recBtn = $('btn-mic-rec');
    var playBtn = $('btn-mic-play');
    var micOn = !!(audio && typeof audio === 'object' && audio.mic_usb);
    var spkOn = !!(audio && typeof audio === 'object' && audio.speaker);

    if (recEl) {
      if (!micOn && !micUi.recording && !micUi.has_recording) {
        setB(recEl, '—', null);
      } else if (micUi.pending === 'start' || micUi.phase === 'starting') {
        setB(recEl, '启动中…', 'warn');
      } else if (micUi.pending === 'stop' || micUi.phase === 'stopping') {
        setB(recEl, '保存中…', 'warn');
      } else if (micUi.recording) {
        setB(recEl, '录音中 ' + Math.floor(micUi.elapsed_s || 0) + 's', false);
      } else if (micUi.playing || micUi.pending === 'play') {
        setB(recEl, '播放中…', 'warn');
      } else if (micUi.has_recording && typeof micUi.duration_s === 'number') {
        setB(recEl, '已录 ' + micUi.duration_s.toFixed(1) + 's', true);
      } else {
        setB(recEl, '未录音', null);
      }
    }

    var busy = micUi.pending !== null || micUi.phase === 'starting' || micUi.phase === 'stopping';
    if (recBtn) {
      var recText = (micUi.pending === 'start' || micUi.phase === 'starting') ? '启动中…'
        : (micUi.pending === 'stop' || micUi.phase === 'stopping') ? '保存中…'
        : micUi.recording ? '结束录音' : '开始录音';
      T(recBtn, recText);
      recBtn.disabled = !micOn || micUi.playing || busy;
      recBtn.classList.toggle('rec-on', !!(micUi.recording && micUi.pending !== 'stop'));
      recBtn.setAttribute('aria-pressed', micUi.recording ? 'true' : 'false');
    }
    if (playBtn) {
      var playingNow = micUi.playing || micUi.pending === 'play';
      T(playBtn, playingNow ? '播放中…' : '播放录音');
      playBtn.disabled = !spkOn || !micUi.has_recording || micUi.recording || micUi.playing || busy;
    }
  }

  /** 写文本：与上次相同则跳过 DOM 写入（蓝图 §5.2.4） */
  function T(el, text) {
    if (!el) return;
    if (el._lastText !== text) {
      el._lastText = text;
      el.textContent = text;
      // 值被省略号截断时给出完整值（hover 才能看到）；没截断就不挂
      // title，免得每个数字都弹出同文案的 tooltip。
      if (el.scrollWidth > el.clientWidth + 1) el.title = text;
      else el.removeAttribute('title');
    }
  }

  function num(v, digits, unit) {
    if (typeof v !== 'number' || !isFinite(v)) return '—';
    var s = v.toFixed(digits);
    if (parseFloat(s) === 0) s = (0).toFixed(digits); // 微小负值别显示成 "-0.0"
    return s + (unit || '');
  }

  function esc(s) {
    return String(s).replace(/[&<>"']/g, function (ch) {
      return { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[ch];
    });
  }

  function led(el, state) {
    if (!el) return;
    var cls = 'led' + (state === 'on' ? ' on' : state === 'warn' ? ' warn' : state === 'bad' ? ' bad' : '');
    if (el._lastLedCls !== cls) { el._lastLedCls = cls; el.className = cls; }
  }

  function setB(el, text, quality) {
    if (!el) return;
    T(el, text);
    var cls = quality === true ? 'ok-t' : quality === false ? 'bad-t' : quality === 'warn' ? 'warn-t' : '';
    if (el._lastMono === undefined) el._lastMono = el.classList.contains('mono');
    if (el._lastBCls !== cls) {
      el._lastBCls = cls;
      el.className = (el._lastMono ? 'mono ' : '') + cls;
    }
  }

  /**
   * 危险按钮的二次确认：第一次点击只把按钮切成「确认态」（文案换成确认
   * 语 + warn 描边），3 秒内再点才执行；超时或失焦自动还原。不弹窗——
   * 极简裁决下危险操作原地确认即可；急停（btn-estop）不走这里，必须一击
   * 生效。返回 true 表示「已执行」，调用方好决定要不要续做。
   */
  function ui_confirm(el, text, onOk) {
    if (!el) return false;
    var armed = el._cfm;
    if (armed) {
      clearTimeout(armed.timer);
      el._cfm = null;
      el.textContent = armed.orig;
      el.classList.remove('cfm');
      el.removeEventListener('blur', armed.reset);
      onOk();
      return true;
    }
    var orig = el.textContent;
    function reset() {
      if (!el._cfm) return;
      clearTimeout(el._cfm.timer);
      el._cfm = null;
      el.textContent = orig;
      el.classList.remove('cfm');
      el.removeEventListener('blur', reset);
    }
    el._cfm = { orig: orig, timer: setTimeout(reset, 3000), reset: reset };
    el.textContent = text;
    el.classList.add('cfm');
    el.addEventListener('blur', reset);
    return false;
  }

  function toast(message, kind) {
    if (!message) return;
    var wrap = $('toasts');
    var el = document.createElement('div');
    el.className = 'toast' + (kind ? ' ' + kind : '');
    el.textContent = message;
    wrap.appendChild(el);
    while (wrap.children.length > 4) wrap.removeChild(wrap.firstChild);
    requestAnimationFrame(function () { el.classList.add('show'); });
    setTimeout(function () {
      el.classList.remove('show');
      setTimeout(function () { if (el.parentNode) el.parentNode.removeChild(el); }, 300);
    }, 3800);
  }

  function requireAuth(action) {
    if (ui.authed) return true;
    toast((action || '该操作') + '需先在「控制」页登录', 'bad');
    return false;
  }

  /* ================= 路由（#/overview /control /map /logs） ================= */

  var PAGE_TITLES = { overview: '概览', control: '控制', logs: '日志' };

  function pageFromHash() {
    var h = location.hash.replace(/^#\/?/, '');
    return PAGE_TITLES[h] ? h : 'overview';
  }

  function applyRoute(page, first) {
    var prev = ui.page;
    ui.page = page;

    var swap = function () {
      for (var key in PAGE_TITLES) {
        var el = $('page-' + key);
        var on = key === page;
        // 蓝图 §5.2.3：非活跃页 content-visibility:hidden（CSS）+ inert（此处）
        el.classList.toggle('active', on);
        if (on) el.removeAttribute('inert'); else el.setAttribute('inert', '');
        var link = document.querySelector('.nav-item[data-page="' + key + '"]');
        if (on) link.setAttribute('aria-current', 'page'); else link.removeAttribute('aria-current');
      }
      T($('page-title'), PAGE_TITLES[page]);
    };

    var focusTitle = function () { $('page-title').focus({ preventScroll: true }); };

    // 丝滑入场：路由切换不包 startViewTransition——VT 的新页快照会定格在
    // card-in 的 backwards 起始帧（透明），0.24s 交叠期间屏幕是空页，快照
    // 一抬卡片才涌出，正是"卡一下"的来源；错落进场本身就是过渡，swap 同步
    // 执行还能让 focusTitle 不再等 VT finished。VT 只留给主题切换（无卡片
    // 重挂载，纯交叉溶解）。
    swap();
    focusTitle();

    C.Spotlight.refresh();
    C.WS.setPage(page);
    if (page === 'logs') fetchLogs();
    if (prev !== page && prev === 'control') releaseAll();
    // 地图已并入控制页第三 tab：切到地图 tab 时补标 MAPTEX（setCtlTab）。
    Store.mark(F.CTRL | F.CONN);
  }

  /* ================= 主题（dark-mode-toggle 规范两态切换） ================= */

  var THEME_KEY = 'c50c-color-scheme';
  var themeMeta = document.querySelector('meta[name="color-scheme"]');

  function systemScheme() {
    return window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
  }

  function renderedScheme() {
    var c = themeMeta.content;
    return (c === 'light dark') ? systemScheme() : c;
  }

  function applyTheme(value) {
    themeMeta.content = value;
    Store.mark(F.THEME);
  }

  function bindTheme() {
    $('theme-toggle').addEventListener('click', function () {
      // 目标 = 当前「渲染值」的反面；与系统一致时不落存储（可被系统拉回）
      var target = renderedScheme() === 'dark' ? 'light' : 'dark';
      var apply = function () {
        try {
          if (target === systemScheme()) localStorage.removeItem(THEME_KEY);
          else localStorage.setItem(THEME_KEY, target);
        } catch (e) { /* 存储被禁仍即时生效 */ }
        applyTheme(target === systemScheme() ? 'light dark' : target);
      };
      // 同路由 swap：隐藏页签回调被推迟会导致 meta color-scheme 与渲染脱节
      if (document.startViewTransition && !document.hidden) document.startViewTransition(apply);
      else apply();
    });
    // 其他标签页改动时同步
    window.addEventListener('storage', function (e) {
      if (e.key === THEME_KEY) applyTheme(e.newValue || 'light dark');
    });
  }

  /* ================= 登录 ================= */

  function setAuthed(on) {
    if (ui.authed === on) return;
    ui.authed = on;
    $('ctl-login').hidden = on;
    $('ctl-main').hidden = !on;
    ['pad-fwd', 'pad-left', 'pad-right', 'pad-rev', 'btn-estop'].forEach(function (id) {
      $(id).disabled = !on;
    });
    if (!on) {
      releaseAll();
      if (ui.gear !== 0) setGear(0);
    } else if (document.activeElement && typeof document.activeElement.blur === 'function') {
      // 登录回车提交后密码框仍持焦点；隐藏表单时必须 blur，否则 window keydown
      // 的 e.target 仍是 INPUT，被 keyTargetIsEditable 拦截导致 WASD 无反应
      document.activeElement.blur();
    }
    // 登录/登出重放错落进场：卡片的 card-in 在页面激活时已播完，不重触发
    // 的话显示区切换（登录卡↔控制件）是一次硬切。批量改 style + 单次回流。
    var scope = on ? $('ctl-main') : $('ctl-login');
    var cards = scope.querySelectorAll('.card');
    for (var i = 0; i < cards.length; i++) cards[i].style.animation = 'none';
    void scope.offsetWidth; /* 一次强制回流让 none 生效，避免逐卡 layout thrash */
    for (i = 0; i < cards.length; i++) cards[i].style.animation = '';
    Store.mark(F.CTRL);
  }

  /* 登录表单提交（控制页与地图页共用）：发 manual_login，回执统一走
   * Store.on('auth' / 'manual_error')。密码框内容与回执错误文案随表单。 */
  function submitLogin(userEl, passEl, errEl) {
    var ok = C.WS.send({
      t: 'manual_login',
      username: userEl.value.trim(),
      password: passEl.value,
    });
    T(errEl, ok ? '' : '未连接到服务');
    if (ok) passEl.value = '';
  }

  /** 登录回执文案（地图已并入控制页，登录卡只剩控制页这张） */
  function showLoginErr(msg) {
    T($('login-err'), msg);
  }

  /* 手动控制内分栏：手动控制 / 传感器设置 / 地图。纯前端切换（不改路由），
   * 三个 pane 靠 hidden 切换（tab 切换不经过 applyRoute） */
  function setCtlTab(name) {
    var manual = name === 'manual';
    var map = name === 'map';
    $('ctl-tab-manual').classList.toggle('active', manual);
    $('ctl-tab-sensors').classList.toggle('active', name === 'sensors');
    $('ctl-tab-map').classList.toggle('active', map);
    $('ctl-tab-manual').setAttribute('aria-selected', String(manual));
    $('ctl-tab-sensors').setAttribute('aria-selected', String(name === 'sensors'));
    $('ctl-tab-map').setAttribute('aria-selected', String(map));
    // roving tabindex：只有当前 tab 在 Tab 序里，方向键切换后焦点跟随
    ['manual', 'sensors', 'map'].forEach(function (n) {
      $('ctl-tab-' + n).tabIndex = n === name ? 0 : -1;
    });
    $('ctl-pane-manual').hidden = !manual;
    $('ctl-pane-sensors').hidden = name !== 'sensors';
    if (map) $('ctl-pane-map').hidden = false;
    else $('ctl-pane-map').hidden = true;
    // 地图 tab 是唯一会渲染地图画布的地方：切进来必须补标 MAPTEX，
    // 否则栅格纹理可能因脏位在非地图帧被丢弃而永不重建（ensureTexture
    // 内部对同 seq 幂等，重复标记无害）。tab 切换不走 applyRoute，这里补。
    if (map) Store.mark(F.MAPTEX);
  }

    /* 侧栏收起/展开：顶栏 #btn-side 切 body.side-collapsed（图标列形态），
   * localStorage 记忆，刷新后保持 */
  function bindSideToggle() {
    var KEY = 'c50c-side-collapsed';
    var apply = function (c) {
      document.body.classList.toggle('side-collapsed', c);
      var b = $('btn-side');
      b.setAttribute('aria-expanded', String(!c));
      b.setAttribute('aria-label', c ? '展开侧栏' : '收起侧栏');
    };
    var saved = null;
    try { saved = localStorage.getItem(KEY); } catch (e) { /* 被禁时默认展开 */ }
    apply(saved === '1');
    $('btn-side').addEventListener('click', function () {
      var c = !document.body.classList.contains('side-collapsed');
      apply(c);
      try { localStorage.setItem(KEY, c ? '1' : '0'); } catch (e) {}
    });
  }

  /* 判读条展开/收起：有异常时整卡可点，展开逐条原因；再点收起。
   * 展开状态 ui.verdictOpen 只存内存（每帧 renderVerdict 重放） */
  function bindVerdictToggle() {
    var card = document.querySelector('#page-overview .card.verdict');
    function toggle() {
      if (!card.classList.contains('expandable')) return;
      ui.verdictOpen = !ui.verdictOpen;
      card.classList.toggle('expanded', ui.verdictOpen);
      card.setAttribute('aria-expanded', String(ui.verdictOpen));
      $('ov-verdict-list').hidden = !ui.verdictOpen;
    }
    card.addEventListener('click', toggle);
    // 键盘可达：Enter / Space 等价于点（role/tabindex 由 renderVerdict 按
    // 是否可展开动态设置，不可展开时不进 Tab 序）。
    // 时序对齐原生 button：Enter 走 keydown（可重复），Space 走 keyup
    // （keydown 只拦滚动），否则按住空格会连发展开/收起。
    card.addEventListener('keydown', function (e) {
      if (e.key === 'Enter') { e.preventDefault(); toggle(); }
      else if (e.key === ' ') e.preventDefault();
    });
    card.addEventListener('keyup', function (e) {
      if (e.key === ' ') toggle();
    });
  }

  function bindCtlTabs() {
    var TAB_NAMES = ['manual', 'sensors', 'map'];
    var tabs = TAB_NAMES.map(function (n) { return $('ctl-tab-' + n); });
    tabs.forEach(function (btn, i) {
      btn.addEventListener('click', function () { setCtlTab(TAB_NAMES[i]); });
      btn.addEventListener('keydown', function (e) {
        var d = e.key === 'ArrowRight' ? 1 : e.key === 'ArrowLeft' ? -1 : 0;
        if (!d) return;
        e.preventDefault();
        var n = (i + d + TAB_NAMES.length) % TAB_NAMES.length;
        tabs[n].focus();
        setCtlTab(TAB_NAMES[n]);
      });
    });
  }

  function bindAuth() {
    $('login-form').addEventListener('submit', function (e) {
      e.preventDefault();
      submitLogin($('login-user'), $('login-pass'), $('login-err'));
    });
    var everAuthed = false;
    Store.on('auth', function (d) {
      if (d.ok) { everAuthed = true; setAuthed(true); toast('已登录', 'ok'); }
      else showLoginErr('用户名或密码错误（或服务端未配置密码）');
    });
    Store.on('manual_error', function (m) {
      var msg = (m && m.message) || '操作被拒';
      if (/请先登录/.test(msg)) {
        setAuthed(false);
        if (ui.page !== 'control') toast('请先在「控制」页登录', 'bad');
        else if (everAuthed) showLoginErr(msg); // 只有会话被踢才提示；未登录时的例行 manual_error 对空登录卡是噪音
        return;
      }
      toast(msg, 'bad');
    });
  }

  /* 手动控制：WASD + 方向键 = 同一批机械方向键（键码拆一套，方向共用） */
  var DIR_BTN = { fwd: null, left: null, right: null, rev: null };
  var DIR_NAME = { fwd: 'forward', left: 'left', right: 'right', rev: 'backward' };
  var KEYBIND = { KeyW: 'fwd', KeyA: 'left', KeyD: 'right', KeyS: 'rev',
                  ArrowUp: 'fwd', ArrowDown: 'rev', ArrowLeft: 'left', ArrowRight: 'right',
                  w: 'fwd', W: 'fwd', a: 'left', A: 'left', d: 'right', D: 'right', s: 'rev', S: 'rev' };
  var pressed = [];   // 保序：最后按下的方向优先
  var hbTimer = 0;

  function isManualLocked() {
    var st = Store.state.status;
    if (st && st.patrol && typeof st.patrol.locked === 'boolean') return st.patrol.locked;
    return !!ui.lockSeen;
  }

  function syncPadHint() {
    var el = $('pad-hint');
    if (!el) return;
    if (isManualLocked()) {
      T(el, '已锁停：请先点击「恢复」，再挂挡操作');
    } else if (!(ui.gear > 0)) {
      T(el, '当前空档：请先挂入巡航档或运动档');
    } else {
      T(el, 'WASD / 方向键移动 · 空格急停');
    }
  }

  function setGear(v) {
    ui.gear = v;
    var gears = document.querySelectorAll('.gear');
    for (var i = 0; i < gears.length; i++) {
      var gv = parseFloat(gears[i].getAttribute('data-v'));
      gears[i].classList.toggle('active', Math.abs(gv - v) < 1e-9);
    }
    if (!(v > 0)) releaseAll();
    else if (pressed.length) syncManual();
    syncPadHint();
  }

  function press(id) {
    if (!ui.authed || DIR_BTN[id].disabled || pressed.indexOf(id) >= 0) return;
    if (isManualLocked()) {
      toast('请先点击「恢复」解除锁停', 'bad');
      return;
    }
    if (!(ui.gear > 0)) {
      toast('当前为空档，请先挂入巡航档或运动档', 'bad');
      return;
    }
    pressed.push(id);
    DIR_BTN[id].classList.add('pressed');
    syncManual();
  }

  function release(id) {
    var i = pressed.indexOf(id);
    if (i < 0) return;
    pressed.splice(i, 1);
    DIR_BTN[id].classList.remove('pressed');
    syncManual();
  }

  function releaseAll() {
    while (pressed.length) release(pressed[pressed.length - 1]);
  }

  /** 立即急停：按钮与空格键共用（等同 btn-estop 行为） */
  function estop() {
    if (!requireAuth('紧急停止')) return;
    releaseAll();
    if (ui.gear !== 0) setGear(0);
    C.WS.send({ t: 'motion_stop' });
  }

  /** 按键集合变化 → manual_start / 心跳 / stop（蓝图 §4.3(1) 手动协议） */
  function syncManual() {
    if (!ui.authed) return;
    if (!pressed.length) {
      if (hbTimer) { clearInterval(hbTimer); hbTimer = 0; }
      C.WS.send({ t: 'manual_stop' });
      return;
    }
    var dir = DIR_NAME[pressed[pressed.length - 1]];
    // 持续模式即唯一驱动方式（定时/巡航 UI 已裁撤）；hold 不得携带 duration（带值即被拒）
    C.WS.send({ t: 'manual_start', id: 1, speed: ui.gear, mode: 'hold', direction: dir });
    if (!hbTimer) hbTimer = setInterval(function () { C.WS.send({ t: 'manual_heartbeat' }); }, 150);
  }

  function bindManual() {
    DIR_BTN.fwd = $('pad-fwd');
    DIR_BTN.left = $('pad-left');
    DIR_BTN.right = $('pad-right');
    DIR_BTN.rev = $('pad-rev');
    Object.keys(DIR_BTN).forEach(function (id) {
      var btn = DIR_BTN[id];
      btn.addEventListener('pointerdown', function (e) { e.preventDefault(); press(id); });
      btn.addEventListener('pointerup', function () { release(id); });
      btn.addEventListener('pointerleave', function () { release(id); });
      btn.addEventListener('pointercancel', function () { release(id); });
    });

    // 输入框/可编辑区里打字不触发方向键；Ctrl/Alt/Meta 修饰键一律不处理；
    // 只在控制页响应（对齐旧 console_page.html 的 `curPage !== 'control'` 门控：
    // authed 跨页持久，地图/概览页误按方向键不能让底盘动、空格也不能急停/弹噪音）
    function keyTargetIsEditable(e) {
      var t = e.target;
      if (!t || (t.closest && t.closest('[hidden]'))) return false;
      return t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.isContentEditable;
    }
    function keyHasModifier(e) {
      return e.ctrlKey || e.altKey || e.metaKey;
    }

    window.addEventListener('keydown', function (e) {
      if (keyHasModifier(e) || keyTargetIsEditable(e) || ui.page !== 'control') return;
      if (e.code === 'Space' || e.key === ' ') {
        e.preventDefault();            // 防滚页
        if (!e.repeat) estop();        // 等同 btn-estop（未登录走 requireAuth 提示）
        return;
      }
      var id = KEYBIND[e.code] || KEYBIND[e.key];
      if (!id || e.repeat) return;
      e.preventDefault();
      press(id);
    });
    window.addEventListener('keyup', function (e) {
      if (keyHasModifier(e) || keyTargetIsEditable(e) || ui.page !== 'control') return;
      var id = KEYBIND[e.code] || KEYBIND[e.key];
      if (id) release(id);
    });
    window.addEventListener('blur', releaseAll);

    // 页面隐藏时若在手动驾驶立刻停车（走现有 stop 路径）
    document.addEventListener('visibilitychange', function () {
      if (document.hidden) releaseAll();
    });

    $('btn-estop').addEventListener('click', estop);

    // 速度档（滑条已撤：三档预设即速度输入，避免双入口；锁停时禁止挂入非空档）
    var gears = document.querySelectorAll('.gear');
    for (var i = 0; i < gears.length; i++) {
      gears[i].addEventListener('click', function (e) {
        var v = parseFloat(e.currentTarget.getAttribute('data-v'));
        if (v > 0 && isManualLocked()) {
          toast('请先点击「恢复」解除锁停，再挂挡', 'bad');
          return;
        }
        setGear(v);
      });
    }
    syncPadHint();

    // 定时/巡航控件已裁撤：手动驾驶 = 方向键 + 三档 + 急停

    $('btn-guide').addEventListener('click', function () {
      ui.guide = !ui.guide;
      $('btn-guide').classList.toggle('active', ui.guide);
      camDirty = true;   // 下一帧相机绘制带上/去掉辅助线
    });

    $('btn-snap').addEventListener('click', function () {
      var cv = $('cam');
      if (!camImg.src || !camImg.complete || !camImg.naturalWidth) { toast('暂无相机帧', 'bad'); return; }
      cv.toBlob(function (blob) {
        if (!blob) { toast('截图失败', 'bad'); return; }
        var a = document.createElement('a');
        a.href = URL.createObjectURL(blob);
        a.download = 'c50c-' + Date.now() + '.png';
        a.click();
        setTimeout(function () { URL.revokeObjectURL(a.href); }, 1000);
      }, 'image/png');
    });
  }

  /* ================= 地图页控件 ================= */

  function setMapMode(mode) {
    ui.mapMode = mode;
    $('m-nav').classList.toggle('active', mode === 'nav');
    $('m-way').classList.toggle('active', mode === 'way');
    T($('map-hint'), mode === 'nav' ? '点击地图下发导航目标' : mode === 'way' ? '点击地图记录航点' : '');
    $('map').classList.toggle('cross', mode !== 'pan');
  }

  /** 地图就绪与新鲜度：栅格断流超过阈值时在画布下方给一行时间提示。
   *  只在翻转时改 DOM，不随 rAF 空转。 */
  var mapStaleWas = null;
  var mapAgeShown = -1;
  function watchMap() {
    var age = Store.mapAgeSec();   // 秒
    var stale = age !== null && age * 1000 > Store.MAP_STALE_MS;
    var el = $('map-age');
    if (!el) return;
    if (stale !== mapStaleWas) {
      mapStaleWas = stale;
      el.hidden = !stale;
      if (!stale) mapAgeShown = -1;
    }
    if (!stale) return;
    var sec = Math.round(age);
    if (sec === mapAgeShown) return;
    mapAgeShown = sec;
    T(el, '地图已 ' + sec + ' 秒未更新');
  }

  /** 没有栅格时选点只会算出无意义的世界坐标：按钮不禁用（禁用后连原因都
   *  点不出来），点一下就明白为什么。 */
  function mapReadyOrExplain() {
    if (C.MapView.ready()) return true;
    toast('地图还没加载，无法选点');
    return false;
  }

  function bindMap() {
    // 地图内联登录卡已随地图并入控制页删除：登录统一走 #login-form。
    $('m-nav').addEventListener('click', function () {
      if (!mapReadyOrExplain()) return;
      setMapMode(ui.mapMode === 'nav' ? 'pan' : 'nav');
    });
    $('m-way').addEventListener('click', function () {
      if (!mapReadyOrExplain()) return;
      setMapMode(ui.mapMode === 'way' ? 'pan' : 'way');
    });
    $('m-clear').addEventListener('click', function () {
      if (!ui.draft.length) return;
      ui_confirm($('m-clear'), '再点确认清空', function () {
        ui.draft.length = 0;
        Store.mark(F.ROUTE);
      });
    });
    $('m-zin').addEventListener('click', function () { C.MapView.zoom(1.3); });
    $('m-zout').addEventListener('click', function () { C.MapView.zoom(1 / 1.3); });
    $('m-fit').addEventListener('click', function () { C.MapView.fit(); });

    Store.on('way_click', function (p) {
      if (!requireAuth('录入航点')) return;
      ui.draft.push({ x: Math.round(p.x * 1000) / 1000, y: Math.round(p.y * 1000) / 1000 });
      Store.mark(F.ROUTE);
    });

    Store.on('nav_click', function (p) {
      if (!requireAuth('导航')) return;
      if (isManualLocked()) {
        toast('当前已锁停，请先点击「恢复」解锁后再导航', 'bad');
        return;
      }
      if (!C.WS.send({ t: 'nav_goal', x: Math.round(p.x * 1000) / 1000, y: Math.round(p.y * 1000) / 1000 })) {
        toast('控制通道未连接，无法导航', 'bad');
      }
    });

    var relocBtn = $('btn-reloc');
    var relocBusyTimer = 0;
    function setRelocBusy(busy) {
      clearTimeout(relocBusyTimer);
      relocBusyTimer = 0;
      if (!relocBtn) return;
      relocBtn.disabled = !!busy;
      relocBtn.textContent = busy ? '重定位中…' : '重定位';
      if (busy) {
        relocBusyTimer = setTimeout(function () { setRelocBusy(false); }, 36000);
      }
    }
    Store.on('relocalize_reply', function () { setRelocBusy(false); });
    Store.on('manual_error', function () { setRelocBusy(false); });
    Store.on('ws_close', function () { setRelocBusy(false); });
    relocBtn.addEventListener('click', function () {
      if (!requireAuth('重定位')) return;
      // 全局扫描-地图匹配重定位（旧栈 relocalize.py 同款）：垫片用 /map 栅格
      // + 最近 /scan 做 360° 三阶段匹配求真实位姿，不依赖当前位姿——这正是
      // AMCL 漂移/丢失时的自救手段。画面若有位姿则附带（后端仅校验记录）。
      var p = Store.poseUsable() ? Store.state.pose : null;
      var msg = { t: 'relocalize' };
      if (p && typeof p.x === 'number' && typeof p.y === 'number' && typeof p.th === 'number') {
        msg.x = Math.round(p.x * 1000) / 1000;
        msg.y = Math.round(p.y * 1000) / 1000;
        msg.yaw = Math.round(p.th * 10000) / 10000;
      }
      if (C.WS.send(msg)) {
        setRelocBusy(true);
        toast('重定位执行中，请稍候…');
      } else {
        toast('控制通道未连接，无法重定位', 'bad');
      }
    });
    $('btn-us-rearm').addEventListener('click', function () {
      if (!requireAuth('重启超声波')) return;
      C.WS.send({ t: 'ultrasonic_rearm' });
    });
    // 传感器重启：服务端白名单映射 systemd --user 单元（相机/雷达同属
    // opi-robot 进程组，底盘独立单元）；回执走 sensor_status → toast
    [['btn-sn-cam', 'camera', '重启相机'], ['btn-sn-lidar', 'lidar', '重启雷达'],
     ['btn-sn-chassis', 'chassis', '重启底盘']].forEach(function (p) {
      $(p[0]).addEventListener('click', function () {
        if (!requireAuth(p[2])) return;
        if (C.WS.send({ t: 'sensor_restart', sensor: p[1] })) {
          toast(p[2] + '指令已下发，恢复需十几秒', 'ok');
        }
      });
    });
    // 喇叭：测试三声 + 音量条（松手才下发，拖动过程只更新读数，避免
    // 每个像素一条命令）。音量持久化在 es8388 的 PCM 控件里，刷新后
    // 由 status.audio.volume 回填。
    $('btn-spk-test').addEventListener('click', function () {
      if (!requireAuth('测试喇叭')) return;
      C.WS.send({ t: 'speaker_test' });
    });
    // 矩阵麦克风：按下录音，再按结束；录制完成可经车体喇叭回放。
    var micRecBtn = $('btn-mic-rec');
    var micPlayBtn = $('btn-mic-play');
    if (micRecBtn) {
      micRecBtn.addEventListener('click', function () {
        if (micUi.pending || micUi.phase === 'starting' || micUi.phase === 'stopping' || micUi.playing) return;
        var stopping = micUi.recording;
        if (!requireAuth(stopping ? '结束录音' : '麦克风录音')) return;
        var cmd = stopping ? 'mic_record_stop' : 'mic_record_start';
        micUi.pending = stopping ? 'stop' : 'start';
        micUi.pendingAt = Date.now();
        if (!C.WS.send({ t: cmd })) {
          micUi.pending = null;
          toast('控制通道未连接，无法操作麦克风', 'bad');
        }
        renderMic(Store.state.status && Store.state.status.audio);
      });
    }
    if (micPlayBtn) {
      micPlayBtn.addEventListener('click', function () {
        if (!micUi.has_recording || micUi.recording || micUi.playing || micUi.pending) return;
        if (!requireAuth('播放录音')) return;
        micUi.pending = 'play';
        micUi.pendingAt = Date.now();
        if (!C.WS.send({ t: 'mic_play' })) {
          micUi.pending = null;
          toast('控制通道未连接，无法播放录音', 'bad');
        }
        renderMic(Store.state.status && Store.state.status.audio);
      });
    }
    function clearMicPending() {
      if (!micUi.pending) return;
      micUi.pending = null;
      renderMic(Store.state.status && Store.state.status.audio);
    }
    Store.on('mic_reply', function (m) {
      micUi.pending = null;
      if (m && m.mic) applyMicSnapshot(m.mic);
      renderMic(Store.state.status && Store.state.status.audio);
    });
    Store.on('manual_error', clearMicPending);
    Store.on('ws_close', clearMicPending);
    var vol = $('spk-vol');
    vol.addEventListener('input', function () {
      spkPend = { v: Number(vol.value), t: Date.now() };
      T($('spk-vol-v'), vol.value + '%');
      vol.setAttribute('aria-valuetext', vol.value + '%');
      syncSpkFill(vol);
    });
    vol.addEventListener('change', function () {
      if (!requireAuth('调节音量')) return;
      spkPend = { v: Number(vol.value), t: Date.now(), sent: true };
      C.WS.send({ t: 'speaker_volume', volume: Number(vol.value) });
    });
    Store.on('sensor_reply', function (m) {
      // 音量命令的回执带板端回读真值（amixer 会取整到原始刻度），
      // 到手立刻定值，不等 5 s 快照；只认最近一次已发送的调整，
      // 拖动中途（sent 已被 input 刷掉）的旧回执不抢。
      // 失败回执 volume 为 null，也要清 pending 让状态帧恢复板端真值
      if (!m) return;
      if (!m.ok) {
        if (spkPend && spkPend.sent) spkPend = null;
        return;
      }
      if (typeof m.volume !== 'number') return;
      if (!spkPend || !spkPend.sent) return;
      spkPend = { v: m.volume, t: Date.now() };
      vol.value = m.volume;
      vol.setAttribute('aria-valuetext', m.volume + '%');
      T($('spk-vol-v'), m.volume + '%');
      syncSpkFill(vol);
    });
    $('btn-nav-home').addEventListener('click', function () {
      if (!requireAuth('回场')) return;
      if (!mapReadyOrExplain()) return;
      if (isManualLocked()) {
        toast('当前已锁停，请先点击「恢复」解锁后再回场', 'bad');
        return;
      }
      if (!C.WS.send({ t: 'nav_goal', x: 0, y: 0, th: 0 })) {
        toast('控制通道未连接，无法回场', 'bad');
      }
    });
    $('btn-nav-cancel').addEventListener('click', function () {
      if (!requireAuth('取消导航')) return;
      C.WS.send({ t: 'nav_cancel' });
    });
    $('btn-patrol-save').addEventListener('click', function () {
      if (!requireAuth('保存路线')) return;
      if (!ui.draft.length) { toast('请先用「录入航点」点选航点', 'bad'); return; }
      C.WS.send({ t: 'patrol_save', points: ui.draft.slice() });
    });
    $('btn-patrol-start').addEventListener('click', function () {
      if (!requireAuth('开始巡航')) return;
      // 巡航一旦启动车会自己动：走二次确认（第一次点击只切确认态）
      ui_confirm($('btn-patrol-start'), '再点确认发车', function () {
        C.WS.send({ t: 'patrol_start' });
      });
    });
    $('btn-motion-stop').addEventListener('click', function () {
      if (!requireAuth('全局停止')) return;
      // 全局停止会断运动许可、车立刻停：误触代价高，走二次确认
      ui_confirm($('btn-motion-stop'), '再点确认全局停止', function () {
        C.WS.send({ t: 'motion_stop' });
      });
    });
    $('btn-motion-rearm').addEventListener('click', function () {
      if (!requireAuth('恢复')) return;
      C.WS.send({ t: 'motion_rearm' });
    });

    Store.on('patrol_reply', function (m) {
      var ok = !!m.ok;
      var msg = m.message || (ok ? '已执行' : '失败');
      T($('patrol-msg'), msg);
      $('patrol-msg').style.color = ok ? '' : 'var(--bad)';
      toast(msg, ok ? 'ok' : 'bad');
      if (ok && Array.isArray(m.route) && m.route.length && ui.draft.length) {
        ui.draft.length = 0;           // 已保存：交由服务端路线展示
        Store.mark(F.ROUTE);
      }
    });
    Store.on('nav_reply', function (m) {
      if (Store.state.status && m && typeof m === 'object') {
        Store.state.status.nav = m;
        Store.mark(F.CTRL | F.ROUTE);
      }
      if (m.error) { toast('导航：' + m.error, 'bad'); return; }
      var state = m.state || '';
      if (state === 'sending') {
        var isHome = m.goal && Math.abs(m.goal.x) < 0.01 && Math.abs(m.goal.y) < 0.01;
        toast(isHome ? '已下发回场导航（0, 0）' : '导航目标已下发', 'ok');
      } else if (state === 'active' || state === 'succeeded') {
        toast('导航：' + navStateText(state), 'ok');
      } else if (state === 'idle') {
        toast(m.result === 'canceled' ? '已取消导航并清除目标' : '已清除导航目标', 'ok');
      }
    });
  }

  /* ================= 日志页 ================= */

  // 筛选状态：时间 / 类型 / 方向 / 关键字。服务端环形缓冲 ≤50 条（<500），
  // 全部在前端即时过滤渲染——无分页、无加载动画。
  var logFilter = { time: 'all', type: '', dir: '', q: '' };

  // 日志抓取：失败不再静默吞掉——页顶挂一行错误 + 「重试」，成功自动撤掉。
  // 失败时不覆盖上一次的数据，避免界面闪空。
  var logErr = '';
  function fetchLogs() {
    var btn = $('btn-log-refresh');
    if (btn) { btn.disabled = true; T(btn, '刷新中…'); }
    function done() {
      if (btn) { btn.disabled = false; T(btn, '刷新'); }
    }
    fetch('/api/wslog')
      .then(function (r) {
        if (!r.ok) throw new Error('HTTP ' + r.status);
        return r.json();
      })
      .then(function (d) {
        logErr = '';
        Store.applyLogs(d);
        renderLogErr();
      })
      .catch(function (e) {
        logErr = '控制帧读取失败（' + (e && e.message ? e.message : '网络不可达') + '）';
        renderLogErr();
      })
      .then(done, done);
  }

  function renderLogErr() {
    var box = $('log-err');
    if (!box) return;
    if (!logErr) { box.hidden = true; box.innerHTML = ''; return; }
    box.hidden = false;
    box.innerHTML = '<div class="err-row"><span></span><button class="btn ghost" type="button">重试</button></div>';
    box.firstChild.firstChild.textContent = logErr;
    box.querySelector('button').addEventListener('click', fetchLogs);
  }

  function hhmmss(unixSec) {
    var d = new Date(unixSec * 1000);
    var p = function (n) { return (n < 10 ? '0' : '') + n; };
    return p(d.getHours()) + ':' + p(d.getMinutes()) + ':' + p(d.getSeconds());
  }

  function logTypeName(it) {
    return (it.detail && it.detail.t) || '(其他)';
  }

  function logMinT() {
    if (logFilter.time === 'today') {
      var d = new Date(); d.setHours(0, 0, 0, 0);
      return d.getTime() / 1000;
    }
    var now = Date.now() / 1000;
    if (logFilter.time === '7d') return now - 7 * 86400;
    if (logFilter.time === '30d') return now - 30 * 86400;
    return 0;
  }

  function syncLogTypes(items) {
    var sel = $('log-type');
    var seen = {};
    for (var i = items.length - 1; i >= 0; i--) seen[logTypeName(items[i])] = true;
    var names = Object.keys(seen).sort();
    var html = '<option value="">全部类型</option>';
    for (i = 0; i < names.length; i++) html += '<option value="' + esc(names[i]) + '">' + esc(names[i]) + '</option>';
    if (sel._lastHtml !== html) {
      sel._lastHtml = html;
      sel.innerHTML = html;
      if (names.indexOf(logFilter.type) < 0) { sel.value = ''; logFilter.type = ''; }
      else sel.value = logFilter.type;
    }
  }

  function renderLogs() {
    var data = Store.state.logs;
    syncLogTypes(data.items);
    var minT = logMinT();
    var q = logFilter.q;
    var shown = 0;
    var html = '';
    for (var i = data.items.length - 1; i >= 0; i--) {
      var it = data.items[i];
      if (minT && it.t < minT) continue;
      if (logFilter.dir && it.kind !== logFilter.dir) continue;
      if (logFilter.type && logTypeName(it) !== logFilter.type) continue;
      if (q && JSON.stringify(it.detail).toLowerCase().indexOf(q) < 0) continue;
      shown++;
      html += '<li><span class="log-t">' + hhmmss(it.t) + '</span>' +
        '<span class="log-kind ' + (it.kind === 'tx' ? 'tx' : 'rx') + '">' + it.kind + '</span>' +
        '<span class="log-detail">' + esc(JSON.stringify(it.detail)) + '</span></li>';
    }
    T($('log-count'), shown + '/' + data.items.length);
    T($('log-age'), data.lastFrameAge == null ? '无帧' : '最后帧 ' + data.lastFrameAge + 's 前');
    var list = $('log-list');
    // 有帧但筛掉光了 ≠ 没有帧：前者提示改条件，后者提示桥没在跑
    if (!shown) {
      html = '<li class="empty-state">' +
        (data.items.length ? '没有符合条件的记录' : '还没有控制帧，检查垫片服务 c50c-web-bridge') +
        '</li>';
    }
    if (list._lastHtml !== html) { list._lastHtml = html; list.innerHTML = html; }
  }

  function bindLogs() {
    $('btn-log-refresh').addEventListener('click', fetchLogs);
    $('log-auto').addEventListener('change', function (e) { ui.autoLog = e.target.checked; });
    document.querySelectorAll('[data-logt]').forEach(function (btn) {
      btn.addEventListener('click', function () {
        document.querySelectorAll('[data-logt]').forEach(function (b) {
          b.classList.toggle('active', b === btn);
        });
        logFilter.time = btn.getAttribute('data-logt');
        renderLogs();
      });
    });
    $('log-type').addEventListener('change', function (e) {
      logFilter.type = e.target.value; renderLogs();
    });
    $('log-search').addEventListener('input', function (e) {
      logFilter.q = e.target.value.trim().toLowerCase(); renderLogs();
    });
    // 更多筛选 popover：外点 / Esc 关闭，关闭后焦点回填触发按钮
    // （键盘用户不能“打开后焦点掉进虚空”）
    function closeMore() {
      var pop = $('log-more-pop');
      if (pop.hidden) return;
      pop.hidden = true;
      $('btn-log-more').setAttribute('aria-expanded', 'false');
    }
    $('btn-log-more').addEventListener('click', function (e) {
      e.stopPropagation();
      var pop = $('log-more-pop');
      var opening = pop.hidden;
      pop.hidden = !opening;
      $('btn-log-more').setAttribute('aria-expanded', String(opening));
      // 打开后把焦点送进 popover（第一个单选项），Esc 才有“回到按钮”的落点
      if (opening) {
        var first = pop.querySelector('input[name="log-dir"]');
        if (first) first.focus();
      } else {
        $('btn-log-more').focus();
      }
    });
    document.addEventListener('click', function (e) {
      var pop = $('log-more-pop');
      if (!pop.hidden && !pop.contains(e.target) && e.target !== $('btn-log-more')) closeMore();
    });
    document.addEventListener('keydown', function (e) {
      if (e.key !== 'Escape') return;
      if ($('log-more-pop').hidden) return;
      e.stopPropagation();
      closeMore();
      $('btn-log-more').focus();
    });
    document.querySelectorAll('input[name="log-dir"]').forEach(function (r) {
      r.addEventListener('change', function (e) {
        logFilter.dir = e.target.value; renderLogs();
      });
    });
    setInterval(function () {
      if (ui.page === 'logs' && ui.autoLog && !document.hidden) fetchLogs();
    }, 2000);
  }

  /* ================= 相机 ================= */

  var camImg = new Image();
  var camDirty = false;
  camImg.onload = function () { camDirty = true; };

  function onImgFrame() {
    var img = Store.state.img;
    if (!img || !img.d) return;
    camImg.src = 'data:' + (img.mime || 'image/png') + ';base64,' + img.d;
  }

  function camFrame() {
    if (!camDirty || ui.page !== 'control') return;
    camDirty = false;
    var cv = $('cam');
    var cx = cv.getContext('2d');
    // 相机物理倒装补偿（2026-11-15 现场确认，旧栈「翻转」同款 rotate(180deg)）：
    // 在画布内旋转而非 CSS 变换，截图走同一 canvas 一起生效；雷达画中画是独立 canvas，不受影响。
    cx.save();
    cx.translate(cv.width, cv.height);
    cx.rotate(Math.PI);
    cx.drawImage(camImg, 0, 0, cv.width, cv.height);
    cx.restore();
    // 辅助线（九宫格 + 十字细线）叠在视频上，默认关；只在开启时画
    if (ui.guide) {
      cx.strokeStyle = 'rgba(231,233,239,0.5)';
      cx.lineWidth = 1;
      for (var i = 1; i < 3; i++) {
        var x = cv.width * i / 3, y = cv.height * i / 3;
        cx.beginPath(); cx.moveTo(x, 0); cx.lineTo(x, cv.height); cx.stroke();
        cx.beginPath(); cx.moveTo(0, y); cx.lineTo(cv.width, y); cx.stroke();
      }
      // 中心十字：比网格线淡一档，不抢视觉
      cx.strokeStyle = 'rgba(231,233,239,0.35)';
      cx.beginPath(); cx.moveTo(cv.width / 2, 0); cx.lineTo(cv.width / 2, cv.height); cx.stroke();
      cx.beginPath(); cx.moveTo(0, cv.height / 2); cx.lineTo(cv.width, cv.height / 2); cx.stroke();
    }
  }

  /* ================= DOM 渲染器（rAF 里按脏位批量执行） ================= */

  var NAV_TEXT = { idle: '待命', active: '导航中', succeeded: '已到达', failed: '失败', canceling: '取消中', canceled: '已取消' };
  function navStateText(s) { return NAV_TEXT[s] || s || '—'; }

  var PATROL_TEXT = { idle: '空闲', running: '巡逻中', paused: '暂停', stopped: '已停止', error: '异常' };

  /* 设备节点全中文：键名→中文标签（概览卡与判读条共用），状态值→在线/离线 */
  var NODE_CN = {
    camera_node: '相机', lidar_node: '激光雷达', c50c_uart3_node: 'ROS 通讯',
    chassis_status_node: '底盘', web_bridge: 'Web 服务'
  };
  var NODE_STATE_TEXT = { online: '在线', offline: '离线' };

  /* 许可/会话状态全中文（服务端枚举 → 操作用语） */
  var LIC_TEXT = {
    ok: '正常', up: '握手中', idle: '待连接', down: '断开',
    session_mismatch: '会话被占用', license_lost: '许可丢失',
    lock_stop: '锁停', unlicensed: '未许可', session_dead: '会话失效',
    license_expired: '许可过期', licensed: '已许可'
  };
  function licText(s) { return LIC_TEXT[s] || s || '—'; }

  /* 巡逻停止/锁停原因（patrol.py::LOCK_REASON_TEXT 的移植；未知原因原样显示） */
  var STOP_CAUSE_TEXT = {
    global_stop: '用户全局停车',
    no_safe_path: '规划连续 3 秒无安全路径',
    nav_failed: '导航失败',
    nav_rejected: '导航目标被拒绝',
    dispatch_failed: '目标发送失败',
    goal_timeout: '目标迟迟未被接受',
    web_disconnected: '网页断线',
    license_lost: '网页许可中断',
    manual_takeover: '手动接管',
    map_changed: '地图已更换，请重新保存路线',
    /* 锁停原因可携带许可侧枚举（patrol.py:821 session_dead），许可表里已有译名 */
    session_dead: '会话失效',
  };
  function causeText(s) { return STOP_CAUSE_TEXT[s] || s; }

  /* 安全门拒绝原因（fusion_node / drive_safety）：弹窗一目了然指向真实来源 */
  var GATE_REASON_TEXT = {
    raw_frames_stalled: '雷达帧中断',
    evidence_stale: '雷达数据过期',
    lidar_connecting: '雷达重连中',
    front_no_points: '前方雷达无回波',
    ultrasonic_latched: '超声波触发锁存',
    obstacle_too_close: '障碍物过近',
  };
  function gateReasonText(s) { return GATE_REASON_TEXT[s] || s; }

  /* WiFi 行海报式文本「103 · -53 dBm」：信号兼容 rssi/signal_dbm。
   * 频段段已去掉——24px 下带频段必折行/截断，且频段信息量最低 */
  /* WiFi 信号强度 → 信号格数（0-4）：经典 RSSI 分档，
   * ≥-55 满格 / ≥-67 三格 / ≥-75 两格 / >-90 一格，采不到 0 格 */
  function netWiFiBars(net) {
    var sig = net.rssi != null ? net.rssi : net.signal_dbm;
    if (typeof sig !== 'number' || !isFinite(sig)) return 0;
    if (sig >= -55) return 4;
    if (sig >= -67) return 3;
    if (sig >= -75) return 2;
    return sig > -90 ? 1 : 0;
  }

  /** 概览卡片首帧前的占位：值位换成微光条，别让一屏「—」看着像坏了。
   * 一旦 status 到达就整卡还原成正常内容（次数由 _skelDone 保证只做一次）。 */
  function overviewSkeleton() {
    document.querySelectorAll('.page[data-page="overview"] .card .kv b').forEach(function (b) {
      if (b._skel) return;
      b._skel = 1;
      b._skelText = b.textContent;
      b.innerHTML = '<span class="skeleton"></span>';
    });
  }

  function renderOverview() {
    var st = Store.state.status;
    if (!st) return;
    // 首帧到达：把骨架占位撤掉（值由下面的 T() 覆盖）
    if (document.querySelector('.page[data-page="overview"] .skeleton')) {
      document.querySelectorAll('.page[data-page="overview"] .kv b').forEach(function (b) {
        if (!b._skel) return;
        b._skel = 0;
        if (b.firstChild && b.firstChild.classList && b.firstChild.classList.contains('skeleton')) {
          b.textContent = b._skelText || '—';
        }
      });
    }

    var modeLabel = st.mode === 'fake' ? '仿真' : '实车';
    if (st.boot && st.boot.initializing) {
      modeLabel += ' · 开机静止 ' + Math.max(0, Math.ceil(st.boot.remain_s)) + 's';
    }
    T($('chip-mode'), modeLabel);

    // 底盘
    var ch = st.chassis || {};
    led($('ov-ch-led'), ch.state === 'fresh' ? 'on' : ch.state === 'stale' ? 'bad' : 'off');
    T($('ov-ch-state'), ch.state === 'fresh' ? '在线' : ch.state === 'stale' ? '超时' : '未知');
    T($('ov-ch-vx'), num(ch.vx_m_s, 1, ' m/s'));
    T($('ov-ch-mileage'), num(ch.mileage_m, 1, ' m'));

    // 电池（6S 标定文案由服务端下发）
    var bat = st.battery || {};
    led($('ov-bat-led'), bat.state === 'ok' ? 'on' : bat.state === 'warn' ? 'warn' : bat.state === 'bad' ? 'bad' : 'off');
    T($('ov-bat-num'), typeof bat.percent === 'number' ? bat.percent + '%' : '—');
    T($('ov-bat-text'), bat.text || '无数据');
    var bar = $('ov-bat-pct');
    if (typeof bat.percent === 'number' && bar._lastPct !== bat.percent) {
      bar._lastPct = bat.percent;
      bar.style.width = bat.percent + '%';
      bar.style.background = bat.charging ? 'var(--accent)'
        : bat.percent < 25 ? 'var(--bad)'
        : bat.percent < 40 ? 'var(--warn)' : 'var(--ok)';
    }

    // 超声波读数不上概览（只作安全门）；设备节点卡只显其健康状态（见下）

    // 系统卡 2×2 圆形仪表：CPU/GPU/内存（环弧=占用，环内百分比）+ 温度
    // （环弧按 0-100℃ 线性、环内 0.1℃ 精度）；≥85% 或 ≥85℃ 整环变红。
    // 磁盘占用已从界面撤除（用户定版，sysinfo 仍采集）。
    var sys = st.sys;
    var SYS_CIRC = 188.4956;
    if (sys && typeof sys === 'object') {
      [['ov-sys-cpu', 'cpu'], ['ov-sys-gpu', 'gpu'],
       ['ov-sys-mem', 'mem']].forEach(function (p) {
        var v = sys[p[1]];
        var ok = typeof v === 'number';
        var b = $(p[0]);
        T(b, ok ? Math.round(v) + '%' : '—');
        var wrap = b.closest('.sys-donut');
        if (!wrap) return;
        var val = wrap.querySelector('.val');
        if (val && val._lastOff !== (ok ? v : null)) {
          val._lastOff = ok ? v : null;
          val.style.strokeDashoffset = (ok ? SYS_CIRC * (1 - Math.min(1, Math.max(0, v / 100))) : SYS_CIRC) + 'px';
        }
        setB(b, b.textContent, ok && v >= 85 ? 'warn' : null);
        wrap.classList.toggle('warn', ok && v >= 85);
      });
      // SoC 温度环（thermal_zone0，0.1℃ 精度；≥75℃ 环变黄、≥85℃ 整环红）
      var temp = sys.temp;
      var tOk = typeof temp === 'number';
      var tb = $('ov-sys-temp');
      T(tb, tOk ? temp.toFixed(1) + '°' : '—');
      var twrap = tb.closest('.sys-donut');
      if (twrap) {
        var tval = twrap.querySelector('.val');
        var frac = tOk ? Math.min(1, Math.max(0, temp / 100)) : 0;
        if (tval && tval._lastOff !== (tOk ? temp : null)) {
          tval._lastOff = tOk ? temp : null;
          tval.style.strokeDashoffset = (SYS_CIRC * (1 - frac)) + 'px';
        }
        twrap.classList.toggle('warn', tOk && temp >= 85);
        twrap.classList.toggle('hot', tOk && temp >= 75 && temp < 85);
      }
      setB(tb, tb.textContent, tOk && temp >= 85 ? false : tOk && temp >= 75 ? 'warn' : null);
      led($('ov-sys-led'), sys.cpu != null ? 'on' : 'off');
    } else {
      ['ov-sys-cpu', 'ov-sys-gpu', 'ov-sys-mem', 'ov-sys-temp'].forEach(function (id) {
        var b = $(id);
        T(b, '—');
        var wrap = b.closest('.sys-donut');
        if (wrap) {
          var val = wrap.querySelector('.val');
          if (val) val.style.strokeDashoffset = SYS_CIRC + 'px';
          wrap.classList.remove('warn', 'hot');
        }
      });
      led($('ov-sys-led'), 'off');
    }

    // 任务（命令行已撤：与底盘卡速度重复）
    var task = st.task || {};
    led($('ov-task-led'), task.state && task.state !== 'idle' ? 'on' : 'off');
    T($('ov-task-text'), task.state === 'idle' ? '空闲' : (task.text || '—'));
    setB($('ov-task-ch'), task.channel ? '就绪' : '断开', !!task.channel);

    // 设备节点（全中文提示）。Web 服务是页面本身，不单独展示——挂了整个页面
    // 都不可能有数据；ovIssues 仍监控它，异常照报。
    var nodes = st.nodes || {};
    [['ov-n-cam', 'camera_node'], ['ov-n-lidar', 'lidar_node'],
     ['ov-n-chassis', 'chassis_status_node'], ['ov-n-uart3', 'c50c_uart3_node']].forEach(function (pair) {
      var v = nodes[pair[1]];
      setB($(pair[0]), v == null ? '—' : (NODE_STATE_TEXT[v] || v), v === 'online' ? true : v === 'offline' ? false : null);
    });
    // 超声波按设备看健康（不显示米数）：正常/锁存/故障
    var usd = st.drive_safety && st.drive_safety.ultrasonic;
    var usBad = !!usd && (usd.latched || !usd.ok);
    setB($('ov-n-us'), !usd ? '—' : usd.latched ? '锁存' : usd.ok ? '正常' : '故障',
      !usd ? null : !usBad);

    // 主机/网络原始转储卡已撤（调试信息不上操作界面）

    renderVerdict(st);
  }

  /* ---- 问题聚合判读：一眼看到"哪张卡有问题"，不逐卡扫。
   * 每条带 detail（点开判读条时显示的具体原因），数据全部来自 status ---- */
  function ovIssues(st) {
    var out = [];
    var ch = st.chassis || {};
    if (ch.state && ch.state !== 'fresh') {
      out.push({ ov: 'ch', text: '底盘' + (ch.state === 'stale' ? '数据超时' : '无数据'),
        detail: ch.state === 'stale'
          ? '底盘遥测 ' + (typeof ch.age === 'number' ? ch.age.toFixed(1) + 's ' : '') + '未更新（阈值 3s）'
          : '未收到底盘遥测帧' });
    }
    var bat = st.battery || {};
    if (bat.state === 'warn' || bat.state === 'bad') {
      out.push({ ov: 'bat', text: (bat.state === 'bad' ? '电池异常' : '低电量') +
        (typeof bat.percent === 'number' ? ' ' + bat.percent + '%' : ''),
        detail: bat.text || '' });
    }
    var usd = st.drive_safety && st.drive_safety.ultrasonic;
    if (usd && (usd.latched || !usd.ok)) {
      // 超声波已并入设备节点卡，异常时高亮该卡
      out.push({ ov: 'nodes', text: usd.latched ? '超声波锁存' : '超声波故障',
        detail: usd.latched ? '近距障碍触发安全门闩，需「重启超声波」恢复'
                            : '超声波读数无效，安全门拒绝行驶' });
    }
    if ((st.task || {}).state === 'error') {
      out.push({ ov: 'task', text: '任务异常', detail: (st.task || {}).text || '巡航任务进入异常状态' });
    }
    // SoC 高温（≥75℃ 预警 / ≥85℃ 过热）：正常不报，异常才上判读条
    var temp = (st.sys || {}).temp;
    if (typeof temp === 'number' && temp >= 75) {
      out.push({ ov: 'sys', text: temp >= 85 ? '系统过热 ' + Math.round(temp) + '°C' : '系统高温 ' + Math.round(temp) + '°C',
        detail: 'SoC 温度 ' + temp.toFixed(1) + ' °C，检查散热与舱内通风' });
    }
    // 许可无独立卡片：只报文案，ov 留空（无对应卡可高亮）
    if (ui.lockSeen) {
      var patrol = st.patrol || {};
      var why = patrol.lock_reason || patrol.stop_reason || patrol.stop_cause;
      out.push({ text: '已锁停',
        detail: '锁停原因：' + (why ? causeText(why) : '未知') });
    }
    if (ui.lic && ui.lic.state && ui.lic.state !== 'ok' && ui.lic.state !== 'idle' && ui.lic.state !== 'up') {
      out.push({ text: '许可异常 ' + licText(ui.lic.state),
        detail: '许可' + licText(ui.lic.state) });
    }
    var nodes = st.nodes || {};
    var off = [];
    Object.keys(NODE_CN).forEach(function (k) {
      if (nodes[k] && nodes[k] !== 'online') off.push(NODE_CN[k]);
    });
    if (off.length) out.push({ ov: 'nodes', text: '节点离线 ' + off.join('/'),
      detail: '离线节点：' + off.join('、') });
    return out;
  }

  function renderVerdict(st) {
    var issues = ovIssues(st || {});
    var bad = issues.length > 0;
    var alarmed = {};
    issues.forEach(function (i) { if (i.ov) alarmed[i.ov] = true; });
    led($('ov-verdict-led'), bad ? 'bad' : 'on');
    T($('ov-verdict-text'), bad ? issues.length + ' 项异常' : '一切正常');
    T($('ov-verdict-detail'), bad ? issues.map(function (i) { return i.text; }).join(' · ')
                                  : '遥测与节点全部在线');
    document.querySelectorAll('#page-overview .card').forEach(function (c) {
      var k = c.getAttribute('data-ov');
      c.classList.toggle('alarming', !!(k && alarmed[k]));
    });
    // 展开区：有异常才可点开（chevron + 可点）；异常清零强制收起。
    // 可展开时才是可聚焦按钮（role=button + tabindex=0），否则不进 Tab 序
    // 也不告诉读屏“可点”。
    var card = $('ov-verdict-list').closest('.verdict');
    card.classList.toggle('expandable', bad);
    if (bad) {
      card.setAttribute('role', 'button');
      card.setAttribute('tabindex', '0');
    } else {
      card.removeAttribute('role');
      card.removeAttribute('tabindex');
    }
    if (!bad) ui.verdictOpen = false;
    card.classList.toggle('expanded', bad && !!ui.verdictOpen);
    card.setAttribute('aria-expanded', String(bad && !!ui.verdictOpen));
    var sig = issues.map(function (i) { return i.text + '|' + i.detail; }).join(';');
    var list = $('ov-verdict-list');
    if (list._lastSig !== sig) {
      list._lastSig = sig;
      list.innerHTML = issues.map(function (i) {
        return '<div class="verdict-item"><b>' + i.text + '</b><span>' + i.detail + '</span></div>';
      }).join('');
    }
    list.hidden = !(bad && !!ui.verdictOpen);
  }

  function renderConn() {
    // 连接 LED：ping RTT 存在即认为 /ws 在线
    var lic = ui.lic;
    var online = ui.rtt != null;
    led($('led-conn'), online ? (lic.state === 'ok' ? 'on' : 'warn') : 'bad');
    T($('chip-rtt'), ui.rtt != null ? ui.rtt + ' ms' : '—');
    var licLabel;
    if (lic.state === 'ok') licLabel = '许可 正常';
    else licLabel = '许可 ' + licText(lic.state);
    T($('chip-lic'), licLabel);
    // 许可不再占概览卡：状态在顶栏 chip，异常/锁停只走判读条文案与登录后的控制页提示
    // 网络卡：控制链路与延迟是客户端实测值；WiFi 行等实车 iw 采集器接入
    var netOn = ui.rtt != null;
    led($('ov-net-led'), netOn ? 'on' : 'bad');
    setB($('ov-net-conn'), netOn ? '在线' : '离线', netOn);
    T($('ov-net-rtt'), netOn ? ui.rtt + ' ms' : '—');
    var net = (Store.state.status || {}).network;
    var hasWifi = net && typeof net === 'object' && net.ssid;
    T($('ov-net-wifi'), hasWifi ? net.ssid : '无数据');
    // 强度不再写 dBm 文本，改四格信号柱（实格 currentColor、空格底色）
    var bars = hasWifi ? netWiFiBars(net) : 0;
    document.querySelectorAll('#ov-net-sig rect').forEach(function (r, i) {
      r.classList.toggle('on', i < bars);
    });
    renderVerdict(Store.state.status);
  }

  function renderCtrl() {
    var st = Store.state.status;
    if (!st) return;

    // 安全门（状态行已撤）：仅在处于非驻车档/正在导航/正在巡逻/有实际运动意图时弹跳变提醒，
    // 静止驻车时不因雷达偶发抖动频繁弹窗刷屏。
    var ds = st.drive_safety || {};
    var gate = ds.allowed === true ? true : ds.allowed === false ? false : null;
    var motionActive = ui.gear !== 0
      || !!(st.nav && st.nav.state === 'active')
      || !!(st.patrol && st.patrol.status === 'running')
      || !!(ds.motion && ds.motion.active);
    if (gate !== ui.gateSeen) {
      if (ui.gateSeen !== undefined && motionActive) {
        if (gate === false) toast('控制被拒' + (ds.reason ? '：' + gateReasonText(ds.reason) : ''), 'bad');
        else if (gate === true) toast('控制已放行', 'ok');
      }
      ui.gateSeen = gate;
    }

    // 相机帧率（雷达已并入相机画中画，无独立卡片）
    T($('cam-fps'), ui.authed && st.image_fps > 0 ? num(st.image_fps, 1, ' fps') : '');

    // 传感器设置卡（全部按「状态」行同格式：可重启的看健康/在线性，
    // 喇叭/矩阵麦克风按 status.audio 看设备在位性，无后台服务不提供重启；
    // 超声波不再显示米数，距离只在安全门内部生效）
    var usd = ds.ultrasonic;
    setB($('sn-us-st'), !usd ? '—' : usd.latched ? '锁存' : usd.ok ? '正常' : '故障',
      !usd ? null : (usd.latched || !usd.ok) ? false : true);
    [['sn-cam-st', 'camera_node'], ['sn-lidar-st', 'lidar_node'],
     ['sn-ch-st', 'chassis_status_node']].forEach(function (pair) {
      var v = (st.nodes || {})[pair[1]];
      setB($(pair[0]), v == null ? '—' : (NODE_STATE_TEXT[v] || v),
        v === 'online' ? true : v === 'offline' ? false : null);
    });
    var audio = st.audio;
    var spkOn = !!(audio && typeof audio === 'object' && audio.speaker);
    if (audio && typeof audio === 'object') {
      setB($('sn-spk-st'), audio.speaker ? '在线' : '未检测到', !!audio.speaker);
      setB($('sn-mic-st'), audio.mic_usb ? '在线' : '未检测到', !!audio.mic_usb);
    } else {
      setB($('sn-spk-st'), '无数据', null);
      setB($('sn-mic-st'), '无数据', null);
    }
    // 喇叭音量：没检测到喇叭就置灰，避免对着空设备调音量自我安慰；
    // 用户正在拖动的这半秒不回写，防止状态帧把滑块弹回去。
    // audio.volume 是板端 5 s 一轮的快照，刚调完音量时还是旧值——
    // spkPend 未清（回执定值 / 状态帧追上 / 8 s 超时）前整块不回写，
    // 否则数字会被旧值压住「等一会儿才变」。
    var spkVol = $('spk-vol');
    var spkBtn = $('btn-spk-test');
    if (spkVol) {
      spkVol.disabled = !spkOn;
      if (spkBtn) spkBtn.disabled = !spkOn;
      var volVal = audio && typeof audio === 'object' ? audio.volume : null;
      if (typeof volVal === 'number') {
        if (spkPend && (volVal === spkPend.v || Date.now() - spkPend.t > 8000)) spkPend = null;
        if (!spkPend) {
          if (document.activeElement !== spkVol) spkVol.value = volVal;
          spkVol.setAttribute('aria-valuetext', volVal + '%');
          syncSpkFill(spkVol);
          T($('spk-vol-v'), volVal + '%');
        }
      } else {
        spkPend = null;
        spkVol.setAttribute('aria-valuetext', '不可用');
        T($('spk-vol-v'), '—');
      }
    }
    if (audio && typeof audio === 'object' && audio.mic && typeof audio.mic === 'object') {
      applyMicSnapshot(audio.mic);
    }
    renderMic(audio);

    var nav = st.nav || {};
    T($('nav-state'), navStateText(nav.state));
    setB($('nav-state'), $('nav-state').textContent,
      nav.state === 'succeeded' ? true : nav.state === 'failed' ? false
        : nav.state === 'active' ? 'warn' : null);
    T($('nav-goal'), nav.goal
      ? nav.goal.x.toFixed(2) + ', ' + nav.goal.y.toFixed(2)
      : '—');

    var patrol = st.patrol || {};
    if (patrol.locked && ui.gear !== 0) setGear(0);
    syncPadHint();
    var pState = patrol.locked ? '已锁停' : (PATROL_TEXT[patrol.status] || patrol.status || '未启动');
    T($('patrol-state'), pState);
    setB($('patrol-state'), pState, patrol.locked ? false : patrol.status === 'running' ? 'warn' : null);

    // 异常详情：当前航点 i/n、停止原因、锁停原因（字段缺失显示「—」）
    var n = typeof patrol.waypoints === 'number' ? patrol.waypoints
      : Array.isArray(patrol.route) ? patrol.route.length : null;
    var i = typeof patrol.index === 'number' ? patrol.index : null;
    T($('patrol-wp'), i != null && n ? (i + 1) + ' / ' + n : '—');
    var stopReason = patrol.stop_reason || patrol.stop_cause;
    T($('patrol-stop'), stopReason ? causeText(stopReason) : '—');
    var lockReason = patrol.lock_reason;
    T($('patrol-lock'), lockReason ? causeText(lockReason) : '—');
  }

  function renderPose() {
    // 过期位姿当没有：垫片 TF 断流后不再发帧，卡片留旧值会让人以为车没动
    var usable = Store.poseUsable();
    var pose = usable ? Store.state.pose : null;
    T($('mp-x'), pose ? num(pose.x, 1, ' m') : '—');
    T($('mp-y'), pose ? num(pose.y, 1, ' m') : '—');
    T($('mp-th'), pose && typeof pose.th === 'number'
      ? (pose.th * 180 / Math.PI).toFixed(1) + '°' : '—');
    var note = $('mp-note');
    if (note) note.hidden = usable;
  }

  /** 位姿新鲜度不在脏位上（断流是「没有新帧」），每帧 rAF 比对一次状态，
   *  只在新鲜↔过期翻转时补一次卡片与画布重绘。 */
  var poseWasUsable = null;
  function watchPose() {
    var now = Store.poseUsable();
    if (now === poseWasUsable) return;
    poseWasUsable = now;
    renderPose();
    C.MapView.invalidate();
  }

  function renderRoute() {
    T($('wp-count'), String(ui.draft.length));
    var html = '';
    for (var i = 0; i < ui.draft.length; i++) {
      html += '<li><span>#' + (i + 1) + '</span><span>' + ui.draft[i].x.toFixed(2) + ', ' + ui.draft[i].y.toFixed(2) + '</span></li>';
    }
    var list = $('wp-list');
    if (list._lastHtml !== html) { list._lastHtml = html; list.innerHTML = html; }
  }

  /* ================= 启动 ================= */

  function boot() {
    C.Spotlight.init();
    // 错落进场：为每页卡片写入页内序号（card-in 的 animation-delay 消费 --i）
    document.querySelectorAll('.page').forEach(function (page) {
      page.querySelectorAll('.card').forEach(function (el, i) {
        el.style.setProperty('--i', Math.min(i, 8));
      });
    });
    bindTheme();
    bindSideToggle();
    bindVerdictToggle();
    bindAuth();
    bindCtlTabs();
    bindManual();
    bindMap();
    bindLogs();

    Scheduler.frame(camFrame);
    Scheduler.frame(C.Radar.create($('radar'), ['control']).frame);
    Scheduler.frame(watchPose);
    Scheduler.frame(watchMap);
    C.MapView.init($('map'));
    Scheduler.frame(C.MapView.frame);

    Scheduler.task(F.STATUS, renderOverview);
    Scheduler.task(F.CTRL, renderCtrl);
    Scheduler.task(F.POSE, renderPose);
    Scheduler.task(F.CONN, renderConn);
    Scheduler.task(F.ROUTE, renderRoute);
    Scheduler.task(F.LOGS, renderLogs);
    Scheduler.task(F.IMG, onImgFrame);

    Store.on('toast', function (d) { toast(d.message, d.kind); });
    Store.on('manual_started', function () { toast('手动控制已接管', 'ok'); });

    window.addEventListener('hashchange', function () { applyRoute(pageFromHash(), false); });
    applyRoute(pageFromHash(), true);
    overviewSkeleton();

    fetchLogs();
    C.WS.boot();
    Scheduler.start();
  }

  boot();
})();
