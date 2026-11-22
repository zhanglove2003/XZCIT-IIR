/* =====================================================================
 * C50C store.js — 响应式内存状态树 + 脏标记位掩码（蓝图 §5.2.4）
 * /ws 与 /ws/video 的 onmessage 只允许写这里（更新内存 + mark 脏位），
 * 绝不直接改 DOM；DOM 与 Canvas 由 scheduler.js 在单一 rAF 节拍里
 * 依据 take() 出来的脏位批量渲染。
 * ===================================================================== */
(function () {
  'use strict';
  var C = (window.C50C = window.C50C || {});

  /** 脏标记位（蓝图 §3.1 store.js：DIRTY_HUD / DIRTY_RADAR / DIRTY_MAP…） */
  var F = {
    STATUS:   1 << 0,  // 新 {t:"status"} 快照（概览 HUD 文本）
    RADAR:    1 << 1,  // 新 {t:"scan"} 点云帧
    MAPTEX:   1 << 2,  // 新 {t:"map"} 栅格帧（仅此时重建 Offscreen 纹理）
    POSE:     1 << 3,  // 新 {t:"pose"} 位姿（仅重绘机器人箭头与航点层）
    IMG:      1 << 4,  // 新 {t:"img"} 相机帧
    CTRL:     1 << 5,  // 控制页派生文本（许可/巡航/导航/输出）
    LOGS:     1 << 6,  // /api/wslog 数据更新
    ROUTE:    1 << 7,  // 航点草稿或巡航路线变化
    THEME:    1 << 8,  // 主题切换（栅格纹理按主题重着色）
    CONN:     1 << 9,  // 连接/许可/RTT 状态变化
  };

  /** 简易事件派发：控制回执（manual_auth / manual_error / 回执帧）用 */
  var listeners = {};

  /** network 快照的「展示签名」：ssid / 信号强度 / ip。
   *  信号强度兼容 rssi 与 signal_dbm 两种字段名（netinfo 双写，前端兼容旧字段）。 */
  function netSig(net) {
    if (!net || typeof net !== 'object') return null;
    return {
      ssid: net.ssid,
      sig: net.rssi != null ? net.rssi : net.signal_dbm,
      ip: net.ip,
    };
  }

  /** 新旧 network 在 ssid / rssi(signal_dbm) / ip 任一上有变化即 true。
   *  两侧都是空/无 network 视为未变化（不白置 F.CONN 空转）；一侧有数据一侧没有算变化。 */
  function netChanged(a, b) {
    var A = netSig(a), B = netSig(b);
    if (!A || !B) return !!A !== !!B;
    return A.ssid !== B.ssid || A.sig !== B.sig || A.ip !== B.ip;
  }

  var state = {
    /** 最新 {t:"status"} 快照（20Hz，服务端 StatusPayload 原样） */
    status: null,
    /** 服务端进程纪元：变化即整页刷新（蓝图 §4.3(2) srv_epoch） */
    srvEpoch: null,
    /** 最新 {t:"scan"}：p=[[deg,r]…] n 有效点 all 总点 hz 实测频率 t 收帧时刻 */
    scan: null,
    /** 最新 {t:"map"}：seq/w/h/res/ox/oy/d(zlib+base64)/age/t */
    map: null,
    /** 最新 {t:"pose"}：x/y/th(m rad) age/t */
    pose: null,
    /** 最新 {t:"img"}：d(base64 png) w/h/fps/mime/t */
    img: null,
    ui: {
      page: 'overview',
      authed: false,        // admin 已登录（服务端按连接记，断线重连需重登）
      rtt: null,            // ping-pong RTT ms
      lic: { state: 'idle', seq: 0, ageMs: null }, // 100ms 许可心跳（Worker）
      gear: 0.4,            // 手动速度 m/s
      mode: 'hold',         // hold | timed
      duration: 2,          // timed 模式时长 s
      draft: [],            // 待保存航点 [{x,y}]（世界坐标）
      mapMode: 'pan',       // pan | nav | way
      autoLog: true,        // 日志页自动刷新
      lockSeen: false,      // status.patrol.locked（全局锁停展示）
      guide: false,         // 相机画面辅助线（九宫格/十字细线）开关，默认关
    },
    /** /api/wslog：{items, lastFrameAge, t} */
    logs: { items: [], lastFrameAge: null, t: 0 },
  };

  var dirty = 0;

  C.Store = {
    FLAGS: F,
    state: state,

    /** 置脏位（WS onmessage / 交互回调里调用） */
    mark: function (flags) { dirty |= flags; },

    /** 取走全部脏位（scheduler 每个 rAF 节拍调用一次） */
    take: function () { var f = dirty; dirty = 0; return f; },

    /** 只读窥视（不取走），供渲染器顺带判断 */
    peek: function () { return dirty; },

    /** 事件：on(evt, fn) / emit(evt, data) */
    on: function (evt, fn) {
      (listeners[evt] || (listeners[evt] = [])).push(fn);
    },
    emit: function (evt, data) {
      var arr = listeners[evt];
      if (!arr) return;
      for (var i = 0; i < arr.length; i++) {
        try { arr[i](data); } catch (e) { console.error('[store] listener', evt, e); }
      }
    },

    /** {t:"status"}：只写内存 + 脏位；srv_epoch 变化触发整页重建（§4.3） */
    applyStatus: function (p) {
      if (state.srvEpoch !== null && p && p.srv_epoch !== state.srvEpoch) {
        state.srvEpoch = p.srv_epoch;
        location.reload();
        return;
      }
      if (p) state.srvEpoch = p.srv_epoch;
      var prevNet = state.status && state.status.network;
      state.status = p || null;
      dirty |= F.STATUS | F.CTRL;
      // WiFi 行（renderConn）只登记在 F.CONN 上；network 展示签名一变就得重绘
      if (netChanged(prevNet, state.status && state.status.network)) dirty |= F.CONN;
      if (p && p.patrol && typeof p.patrol.locked === 'boolean') {
        state.ui.lockSeen = p.patrol.locked;
      }
    },

    applyScan: function (f) {
      state.scan = { p: f.p, n: f.n, all: f.all, hz: f.hz, t: performance.now() };
      dirty |= F.RADAR;
    },

    /** {t:"map"}：记录原始帧；zlib+base64 解码与纹理重建由 canvas-map.js 做 */
    applyMap: function (f) {
      state.map = {
        seq: f.seq, w: f.w, h: f.h, res: f.res, ox: f.ox, oy: f.oy,
        d: f.d, age: f.age, t: performance.now(),
      };
      dirty |= F.MAPTEX;
    },

    applyPose: function (f) {
      if (!f || !f.pose) return;
      state.pose = {
        x: f.pose.x, y: f.pose.y, th: f.pose.th,
        age: f.pose_age, t: performance.now(),
      };
      dirty |= F.POSE;
    },

    applyImg: function (f) {
      state.img = { d: f.d, w: f.w, h: f.h, fps: f.fps, mime: f.mime, t: performance.now() };
      dirty |= F.IMG;
    },

    applyLogs: function (data) {
      state.logs = {
        items: (data && data.items) || [],
        lastFrameAge: data ? data.last_frame_age : null,
        t: performance.now(),
      };
      dirty |= F.LOGS;
    },
  };
})();
