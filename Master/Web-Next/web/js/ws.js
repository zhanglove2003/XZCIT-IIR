/* =====================================================================
 * C50C ws.js — /ws 与 /ws/video 双通道管理 + Blob Web Worker 100ms 许可心跳
 * 蓝图 §4.3 / §3：
 *  - /ws 控制通道：握手收 motion_license_session；1Hz view 心跳；
 *    ping/pong 测 RTT；上行控制消息；下行 status 只进 Store。
 *  - motion_license 由独立 Blob Worker 每 100ms 在自己的 /ws 连接上发送
 *    （seq 严格递增，ttl 350ms），主线程卡顿不影响许可续期。
 *  - /ws/video 媒体通道：仅在 control / map 页开启，消息只进 Store。
 * ===================================================================== */
(function () {
  'use strict';
  var C = (window.C50C = window.C50C || {});
  var Store = C.Store;
  var F = Store.FLAGS;

  var VIEW_BEAT_MS = 1000;   // 蓝图 §4.3：view 心跳 1Hz
  var PING_MS = 5000;        // RTT 探测
  var RECONNECT_MS = 1500;   // 断线重连间隔
  var LICENSE_MS = 100;      // Worker 许可心跳周期
  var LICENSE_TTL_MS = 350;
  var LIC_SESSION = Date.now().toString(16) + Math.random().toString(16).slice(2, 10);

  var ui = Store.state.ui;

  function wsURL(path) {
    return (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + path;
  }

  function connChanged() { Store.mark(F.CONN); }

  function viewPage() {
    // 蓝图 §4.3(1)：page 取值 overview|control|map|idle；logs 页不占实时会话
    return ui.page === 'logs' ? 'idle' : ui.page;
  }

  /* ------------------------------------------------------------------
   * /ws 控制通道
   * ------------------------------------------------------------------ */

  var ctrl = null;        // WebSocket
  var beatTimer = 0;
  var pingTimer = 0;
  var hbTimer = 0;        // Worker 不可用时的主线程许可兜底心跳

  function sendView() {
    sendRaw({ t: 'view', page: viewPage() });
  }

  function sendRaw(obj) {
    if (obj && obj.t === 'manual_login' && !obj.session_id) {
      obj.session_id = LIC_SESSION;
    }
    if (ctrl && ctrl.readyState === 1) {
      ctrl.send(JSON.stringify(obj));
      return true;
    }
    return false;
  }

  function connectCtrl() {
    var s;
    try { s = new WebSocket(wsURL('/ws')); } catch (e) { setTimeout(connectCtrl, RECONNECT_MS); return; }
    ctrl = s;
    connChanged();

    s.onopen = function () {
      connChanged();
      sendView();
      clearInterval(beatTimer);
      beatTimer = setInterval(sendView, VIEW_BEAT_MS);
      clearInterval(pingTimer);
      sendRaw({ t: 'ping', ts: Date.now() });   // 立即测一次 RTT，LED 即刻点亮
      pingTimer = setInterval(function () { sendRaw({ t: 'ping', ts: Date.now() }); }, PING_MS);
    };

    s.onmessage = function (e) { onCtrlMsg(e); };

    s.onerror = function () { try { s.close(); } catch (err) { /* ignore */ } };

    s.onclose = function () {
      ctrl = null;
      clearInterval(beatTimer);
      clearInterval(pingTimer);
      ui.rtt = null;
      connChanged();
      Store.emit('ws_close');
      setTimeout(connectCtrl, RECONNECT_MS);
    };
  }

  function onCtrlMsg(e) {
    var m;
    try { m = JSON.parse(e.data); } catch (err) { return; }
    if (!m || typeof m.t !== 'string') return;

    switch (m.t) {
      case 'status':
        Store.applyStatus(m);              // 只写内存 + 脏位（§5.2.4）
        break;
      case 'motion_license_session':
        // 服务端握手/ack 帧；许可会话号由 Worker（或兜底）自行生成，不在此采纳
        if (!hbTimer && !C.WS.workerOk) startFallbackLicense();
        break;
      case 'pong':
        if (typeof m.ts === 'number') {
          ui.rtt = Math.max(0, Math.round(Date.now() - m.ts));
          connChanged();
        }
        break;
      case 'motion_license_nak':
        ui.lic.state = m.reason || 'nak';
        connChanged();
        Store.emit('toast', { kind: 'bad', message: '运动许可被拒：' + (m.reason || '') });
        break;
      case 'manual_auth':
        Store.emit('auth', { ok: !!m.ok });
        break;
      case 'manual_started':
        Store.emit('manual_started', m);
        break;
      case 'manual_error':
        Store.emit('manual_error', m);
        break;
      case 'ultrasonic_status':
        Store.emit('toast', { kind: m.ok ? 'ok' : 'bad', message: m.message || '' });
        break;
      case 'relocalize_status':
        Store.emit('toast', { kind: m.ok ? 'ok' : 'bad', message: m.message || '' });
        Store.emit('relocalize_reply', m);
        break;
      case 'sensor_status':
        // 通用 toast 之外再广播整帧（音量回执带板端回读值，app.js 用它定值）
        Store.emit('toast', { kind: m.ok ? 'ok' : 'bad', message: m.message || '' });
        Store.emit('sensor_reply', m);
        break;
      case 'mic_status':
        if (m.message) {
          Store.emit('toast', { kind: m.ok ? 'ok' : 'bad', message: m.message });
        }
        Store.emit('mic_reply', m);
        break;
      case 'patrol_status':
        Store.emit('patrol_reply', m);
        break;
      case 'nav_status':
        Store.emit('nav_reply', m);
        break;
      default:
        break;
    }
  }

  /* ------------------------------------------------------------------
   * 100ms 许可心跳：首选 Blob Web Worker（独立线程 / 独立连接）
   * 会话协议（对齐旧 console_page.html 的语义，兼容本后端 license_accept）：
   *  - 会话号由页面随机生成（非服务端下发号）：服务端 license.session 里的
   *    旧值（上一页崩溃残留）在 350ms 失效后被本页 Takeover，序号从 0 重来；
   *  - seq 在页面生命周期内严格递增，WS 重连不清零 —— 否则服务端按
   *    seq_replay 永久拒绝（license_accept 对同会话不重置 last_seq）；
   *  - 被其他活页占用（session_mismatch）时继续按 100ms 节拍发送，旧页
   *    停发 350ms 后自动接管；
   *  - 页面隐藏即发 motion_license_release 交还许可并断开许可连接，回前台
   *    自动重连接管——许可权随前台页（服务端 release 只清会话不锁停）。
   * ------------------------------------------------------------------ */

  var worker = null;

  /* Worker 源码：自己连 /ws，每 100ms 发一条 motion_license。
   * 注意：Blob Worker 里 location 是 blob: 假 URL（host 为空），不能用
   * location.host 拼 WS 地址 —— 基址由主线程在构造 Blob 时注入。 */
  var WORKER_SRC =
    '"use strict";\n' +
    'var WS_BASE = ' + JSON.stringify(wsURL('')) + ';\n' +
    'var ws = null, seq = 0, delay = 800, stopped = false, lastAck = 0, licPaused = false;\n' +
    'var session = ' + JSON.stringify(LIC_SESSION) + ';\n' +
    'function post(m) { postMessage(m); }\n' +
    'function conn() {\n' +
    '  if (stopped) return;\n' +
    '  var s;\n' +
    '  try { s = new WebSocket(WS_BASE + "/ws"); }\n' +
    '  catch (e) { setTimeout(conn, delay); return; }\n' +
    '  ws = s;\n' +
    '  s.onopen = function () { delay = 800; post({ type: "lic", state: "up" }); };\n' +
    '  s.onmessage = function (e) {\n' +
    '    try {\n' +
    '      var m = JSON.parse(e.data);\n' +
    '      if (m.t === "motion_license_session") {\n' +
    '        if (m.session_id === session) { lastAck = Date.now(); post({ type: "lic", state: "ok", seq: seq, age: 0 }); }\n' +
    '        /* 其余为服务端握手帧/他号广播：忽略，不采纳其会话号 */\n' +
    '      } else if (m.t === "motion_license_nak") {\n' +
    '        post({ type: "lic", state: m.reason || "nak" });\n' +
    '      }\n' +
    '    } catch (err) {}\n' +
    '  };\n' +
    '  s.onerror = function () { try { s.close(); } catch (err) {} };\n' +
    '  s.onclose = function () {\n' +
    '    ws = null; lastAck = 0;\n' +
    '    /* 会话号与 seq 保持不变：重连首帧不会被判 seq_replay */\n' +
    '    post({ type: "lic", state: "down" });\n' +
    '    if (!stopped) { delay = Math.min(delay * 2, 5000); setTimeout(conn, delay); }\n' +
    '  };\n' +
    '}\n' +
    'setInterval(function () {\n' +
    '  if (!licPaused && ws && ws.readyState === 1) {\n' +
    '    seq++;\n' +
    '    ws.send(JSON.stringify({ t: "motion_license", session_id: session, seq: seq, sent_ms: Date.now(), ttl_ms: ' + LICENSE_TTL_MS + ' }));\n' +
    '  }\n' +
    '}, ' + LICENSE_MS + ');\n' +
    'setInterval(function () { if (lastAck) post({ type: "lic", state: "ok", seq: seq, age: Date.now() - lastAck }); }, 1000);\n' +
    'onmessage = function (e) {\n' +
    '  if (e.data && e.data.type === "stop") { stopped = true; try { ws && ws.close(); } catch (err) {} }\n' +
    '  else if (e.data && e.data.type === "gate") {\n' +
    '    /* 前台/后台门控：隐藏 = 发 motion_license_release 交还许可并断开本连接\n' +
    '     * （许可权随前台页——否则隐藏残留页会永远心跳占着许可，前台页被\n' +
    '     * session_mismatch 永久拒绝）；回前台立即重连，≤100ms 心跳即可接管。 */\n' +
    '    licPaused = !!e.data.v;\n' +
    '    if (licPaused) {\n' +
    '      try { if (ws && ws.readyState === 1) ws.send(JSON.stringify({ t: "motion_license_release", session_id: session })); } catch (err) {}\n' +
    '      try { ws && ws.close(); } catch (err) {}\n' +
    '      ws = null;\n' +
    '    } else if (!stopped && !ws) { conn(); }\n' +
    '  }\n' +
    '};\n' +
    'conn();\n';

  function startWorker() {
    if (typeof Worker === 'undefined' || typeof Blob === 'undefined') return false;
    try {
      var blob = new Blob([WORKER_SRC], { type: 'text/javascript' });
      worker = new Worker(URL.createObjectURL(blob));
      /* 初始门控：页面可能以隐藏态加载（后台恢复的页签），先同步状态 */
      worker.postMessage({ type: 'gate', v: document.hidden });
      worker.onmessage = function (e) {
        var m = e.data || {};
        if (m.type !== 'lic') return;
        ui.lic.state = m.state;
        if (typeof m.seq === 'number') ui.lic.seq = m.seq;
        if (typeof m.age === 'number') ui.lic.ageMs = m.age;
        connChanged();
      };
      worker.onerror = function () {
        ui.lic.state = 'worker_error';
        connChanged();
      };
      return true;
    } catch (e) {
      return false;
    }
  }

  /** Worker 不可用时的兜底：主线程 100ms 直发（仍优于没有许可）。
   *  会话号与 seq 语义与 Worker 相同：页内随机、严格递增、重连不清零。
   *  同样按可见性门控：隐藏=发一次 release 并停发，回前台恢复节拍。 */
  function startFallbackLicense() {
    var seq = 0;
    var session = LIC_SESSION;
    var bgSent = false;
    hbTimer = setInterval(function () {
      if (document.hidden) {
        if (!bgSent) {
          bgSent = true;
          if (ctrl && ctrl.readyState === 1) {
            sendRaw({ t: 'motion_license_release', session_id: session });
          }
          ui.lic.state = 'down';
          connChanged();
        }
        return;
      }
      bgSent = false;
      if (ctrl && ctrl.readyState === 1) {
        seq++;
        sendRaw({
          t: 'motion_license', session_id: session,
          seq: seq, sent_ms: Date.now(), ttl_ms: LICENSE_TTL_MS,
        });
        ui.lic.state = 'ok';
        ui.lic.seq = seq;
        ui.lic.ageMs = 0;
        connChanged();
      }
    }, LICENSE_MS);
  }

  /* ------------------------------------------------------------------
   * /ws/video 媒体通道（control / map 页开启；标签页隐藏时挂起）
   * ------------------------------------------------------------------ */

  var video = null;
  var videoWant = false;
  var videoTimer = 0;

  function connectVideo() {
    // 不以 document.hidden 为门：后台页签仍收帧入 Store（画布绘制自然停拍），
    // 与旧实现一致；页签隐藏→可见的挂起/恢复由 visibilitychange 处理。
    if (!videoWant || video) return;
    var s;
    try { s = new WebSocket(wsURL('/ws/video')); } catch (e) { scheduleVideo(); return; }
    video = s;
    connChanged();

    s.onopen = function () { connChanged(); };
    s.onmessage = function (e) { onVideoMsg(e); };
    s.onerror = function () { try { s.close(); } catch (err) { /* ignore */ } };
    s.onclose = function () {
      video = null;
      connChanged();
      if (videoWant) scheduleVideo();
    };
  }

  function scheduleVideo() {
    clearTimeout(videoTimer);
    videoTimer = setTimeout(connectVideo, RECONNECT_MS);
  }

  function closeVideo() {
    clearTimeout(videoTimer);
    videoWant = false;
    if (video) {
      var s = video;
      video = null;
      try { s.close(); } catch (err) { /* ignore */ }
    }
    connChanged();
  }

  function onVideoMsg(e) {
    var m;
    try { m = JSON.parse(e.data); } catch (err) { return; }
    switch (m.t) {
      case 'scan': Store.applyScan(m); break;   // 10Hz 点云
      case 'map':  Store.applyMap(m);  break;   // 1Hz/seq 变化 栅格
      case 'pose': Store.applyPose(m); break;   // 5Hz 位姿
      case 'img':  Store.applyImg(m);  break;   // 已登录 control 页相机帧
      default: break;
    }
  }

  document.addEventListener('visibilitychange', function () {
    /* 许可门控：隐藏=交还许可（服务端清会话不锁停），回前台重连接管 */
    if (worker) worker.postMessage({ type: 'gate', v: document.hidden });
    if (document.hidden) {
      // 隐藏页签：挂起视频通道（rAF 已停，收帧无意义），回前台立即重连
      var want = videoWant;
      if (want) { closeVideoInternal(); }
      videoWant = want;
    } else if (videoWant) {
      connectVideo();
    }
  });

  function closeVideoInternal() {
    clearTimeout(videoTimer);
    if (video) {
      var s = video;
      video = null;
      try { s.close(); } catch (err) { /* ignore */ }
    }
    connChanged();
  }

  /* ------------------------------------------------------------------
   * 对外接口
   * ------------------------------------------------------------------ */

  C.WS = {
    send: sendRaw,
    workerOk: false,
    session: LIC_SESSION,

    /** 启动控制通道 + 许可心跳 Worker（app.js boot 时调用一次） */
    boot: function () {
      C.WS.workerOk = startWorker();
      connectCtrl();
    },

    /** 路由切换：立即发 view，并按页面开关视频通道
     *  （地图已并入控制页，视频通道只随控制页开关） */
    setPage: function (page) {
      sendView();
      var want = page === 'control';
      if (want && !videoWant) {
        videoWant = true;
        connectVideo();
      } else if (!want && videoWant) {
        closeVideo();
      }
    },
  };
})();
