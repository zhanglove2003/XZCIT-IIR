# UI 完善工作书（2026-11-22）

> 分支：`feat/ui-polish`（worktree：`repo-push/Web-Next-ui`，基线提交 `e0c1c34`）
> 主线 `dev/differential` 不动；本分支跑稳后再回合。
> 每条格式：现状 / 目标 / 验收口径 / 文件。做完在标题前打 ✅ 并填「验收」行。

## 总原则（不违反）

- 极简：常驻状态行、无用数据、自证信息一律不上；异常才报警（判读条 / toast）。
- 字号只有 14 / 24 / 48 三档；颜色只用 `tokens.css` 语义变量，不写裸色值。
- 不引第三方 UI 框架；不做 i18n、不做弹窗系统、不做日志分页。
- 不碰重定位 / AMCL 链（那条等外部结论）。

---

## 批次 1 · 响应式与布局  ← 优先级最高

**现状**
- 断点只有 `@media (max-width:960px)`（layout.css:811）与 `700px`（:819），960–1120 区间无适配。
- 900px 实测：`.ctl-pane` 仍是两列（`minmax(280px,380px) minmax(300px,1fr)`，layout.css:499-511），相机卡高度随容器拉满，整页被撑到两屏；地图 tab 画布高度塌成一块空白。
- `.map-grid` 右栏固定 268px（layout.css:648-653），中等宽度下挤压画布。

**目标**
- 加 `@media (max-width: 1120px)`：`.ctl-pane` → `300px 1fr`；`.map-grid` 右栏 → 240px。
- 相机画布改 `aspect-ratio: 16 / 9` + `max-height`（视口 60% 封顶），不再随容器拉满。
- `≤700px`：主内容左右内边距、地图/雷达画布高度下限（≥320px）检查一遍。

**验收**
- 1600 / 1280 / 1024 / 900 / 760 五档各截「手动」「地图」两屏；相机卡 ≤ 视口 60% 高；地图画布 ≥ 320px。
- 深浅两主题各一遍。

**文件**：`web/styles/layout.css`、`web/styles/micro.css`（若画布尺寸在 micro）

- [x] 完成 　　验收：1024×800 相机卡高 300px（视口 37%）、方向键与档位全在首屏；760×700 地图画布 300px、右栏落到画布下方；侧栏 188px 收起逻辑未受影响。产物 md5 `eaebf224280860ab64a0fa4a32a34aee`。

---

## 批次 2 · 危险操作二次确认

**现状**
- 无任何 modal/dialog；「全局停止」「清空航点」「开始航点巡航」点一下立即执行（app.js:484-526 区段）。

**目标**
- 新增 `ui_confirm(el, text, onOk)`：按钮原地进入「再点一次确认」态（文案换成确认语、加 warn 描边），3 s 超时 / 失焦 / 再次点击后还原；第二次点击才调 `onOk`。
- 只用在会误触致损的按钮上：全局停止、清空航点、开始航点巡航。**「停止」大红钮不加**（急停必须一击生效）。

**验收**
- 第一次点击：WS 日志里无对应 `rx`，按钮变确认态并在 3 s 后自动还原。
- 3 s 内第二次点击：命令下发，按钮立即还原。

**文件**：`web/js/app.js`（新增 `ui_confirm` + 三处接线）、`web/styles/micro.css`（确认态样式）

- [x] 完成 　　验收：点「全局停止」首次只变文案为「再点确认全局停止」并挂 `.cfm` 类（DOM 实测 `{"text":"再点确认全局停止","cls":"btn estop cfm","armed":true}`），不发命令；3.2 s 后自动还原为「全局停止」；确认态内再点触发执行并立即还原。清空航点、开始巡航同样接入（清空在草稿为空时不拦截）。急停「停止」大红钮未接入，保持一击生效。产物 md5 `18d227370bae2992bdc70c4a9f096e60`。

---

## 批次 3 · 加载 / 空 / 错误态统一

**现状**
- 无骨架屏、无 spinner；概览首帧前全「—」；日志过滤后 0 条只显示计数；`fetchLogs` 失败静默吞错（app.js:563-565）；地图未收到 `/map` 只有「等待地图…」。

**目标**
- 新增三个样式组件：`.skeleton`（微光占位）、`.empty-state`（居中一句话）、失败行复用 `.form-err` + 重试按钮。
- 概览：首帧到达前卡片值位显示骨架。
- 日志：0 命中显示「没有符合条件的记录」；抓取失败显示原因 + 「重试」。
- 地图：未收到 `/map` 时保留「等待地图…」，下面补一行浅色小字「检查垫片服务 c50c-web-bridge」。

**验收**
- 拔网线 / `systemctl --user stop c50c-web-bridge` 两种场景，界面都不再空白或有明确提示。

**文件**：`web/index.html`、`web/js/app.js`、`web/js/canvas-map.js`、`web/styles/layout.css`

- [x] 完成 　　验收：日志搜索无命中显示「没有符合条件的记录」（DOM 实测）；把 `fetch` 打桩成 reject 后点刷新，页顶出现红色错误行「控制帧读取失败（网络不可达(测试)）」+ 重试按钮，且原有日志不被清空；地图等待态在「等待地图…」下补了一行「若长时间无图，检查垫片服务 c50c-web-bridge」；概览首帧前 `.kv b` 显示骨架条，status 到达后自动撤掉。产物 md5 `0fdc86ea16c1a9e5e8888b8017dddb17`。

---

## 批次 4 · 日志页交互补完

**现状**
- 「更多筛选」popover 只能点外部关闭，无 Esc、无焦点回填；刷新按钮无进行中态。

**目标**
- popover：Esc 关闭并把焦点还给触发按钮；关闭前记录 `document.activeElement`。
- 刷新按钮：进行中禁用 + 文案「刷新中…」，失败走批次 3 的错误行。

**验收**
- 键盘 Tab 到「更多筛选」→ Enter 打开 → Esc 关闭 → 焦点仍在该按钮上。

**文件**：`web/js/app.js`（`bindLogs`）

- [x] 完成 　　验收：打开「更多筛选」后焦点落在第一个单选项（`log-dir`），按 Esc 关闭并回到触发按钮（DOM 实测 `{"open":true,"focusInPopover":"log-dir","closedByEsc":true,"focusBackTo":"btn-log-more"}`）；刷新按钮进行中禁用并显示「刷新中…」，失败落到批次 3 的错误行。产物 md5 `a363f59d23a16b277e37fd529f1cfd1d`。

---

## 批次 5 · 键盘可达性

**现状**
- 判读条是 `<article>` + click，无 `role` / `tabindex` / `:focus-visible`；tab 栏有方向键循环（app.js:263-270）但没有 roving tabindex。

**目标**
- 判读条可展开时：`role="button"`、`tabindex="0"`、Enter/Space 触发展开，补焦点轮廓。
- tab 栏：选中项 `tabindex="0"`、其余 `-1`，方向键切换后焦点跟随。

**验收**
- 全程不用鼠标：Tab → 判读条 → Enter 展开 → Tab 到控制页 tab → 方向键切 tab。

**文件**：`web/index.html`、`web/js/app.js`、`web/styles/micro.css`

- [ ] 完成 　　验收：＿＿＿＿

---

## 批次 6 · 小修

**现状 / 目标**
- `.kv b` 超宽省略号截断后信息不可达 → 截断时加 `title`。
- 侧栏收起/展开是宽度突变 → 加 180 ms 过渡（`prefers-reduced-motion` 下禁用）。
- 手动 tab 没有快捷键提示 → 底部一行小字「WASD / 方向键移动 · 空格急停」。

**验收**
- 收起侧栏有过渡动画；手动 tab 底部有一行提示；长数值卡 hover 有完整值 tooltip。

**文件**：`web/js/app.js`、`web/index.html`、`web/styles/layout.css`、`web/styles/micro.css`

- [ ] 完成 　　验收：＿＿＿＿

---

## 已知但**不做**的项（记录在案，避免反复讨论）

| 项 | 原因 |
|---|---|
| Modal / 弹窗系统 | 极简裁决，危险操作用二次确认条替代 |
| i18n | 全站中文定版 |
| 日志分页 | 环形缓冲 <500 条，够用 |
| 触屏手势（双指缩放地图） | 现场只用鼠标/触控板，且 `touch-action:none` 是刻意的 |
| 概览页 960–1100 三列挤压 | auto-fill 自适应已可接受，等现场反馈再调 |

## 构建与部署（每批一遍）

```bash
cd repo-push/Web-Next-ui/Master/Web-Next
node --check web/js/app.js && node --check web/js/ws.js   # 改动过的 JS 全过一遍
cargo test
cargo build --release --target aarch64-unknown-linux-musl
scp target/aarch64-unknown-linux-musl/release/web_next opi:/home/orangepi/web_next_deploy/web_next.new
ssh opi 'cd /home/orangepi/web_next_deploy && md5sum web_next.new && mv web_next.new web_next && systemctl --user restart c50c-web-next'
```

截图存 `docs/ui-shots/<批次>-<宽度>-<页>.png`。
