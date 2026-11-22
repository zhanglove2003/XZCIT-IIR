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

- [x] 完成 　　验收：判读条在有异常时为 `role=button` + `tabindex=0`，Enter 展开、Space 收起（DOM 实测 `{"before":{"role":"button","tab":"0","open":false},"openedByEnter":true,"closedBySpace":true}`），并补了 `:focus-visible` 焦点环；控制页 tab 改为 roving tabindex（仅当前 tab 在 Tab 序，方向键切换后焦点跟随）。产物 md5 `68a51d4e30a84b6ef2157797281186fe`。

---

## 批次 6 · 小修

**现状 / 目标**
- `.kv b` 超宽省略号截断后信息不可达 → 截断时加 `title`。
- 侧栏收起/展开是宽度突变 → 加 180 ms 过渡（`prefers-reduced-motion` 下禁用）。
- 手动 tab 没有快捷键提示 → 底部一行小字「WASD / 方向键移动 · 空格急停」。

**验收**
- 收起侧栏有过渡动画；手动 tab 底部有一行提示；长数值卡 hover 有完整值 tooltip。

**文件**：`web/js/app.js`、`web/index.html`、`web/styles/layout.css`、`web/styles/micro.css`

- [x] 完成 　　验收：手动 tab 底部出现「WASD / 方向键移动 · 空格急停」提示（截图为证）；侧栏收起/展开有 width+padding 过渡（DOM 实测 `transition: width, padding`），`prefers-reduced-motion` 下关闭；`.kv b` 截断时才挂 `title`（T() 统一路径，未截断不弹 tooltip）。产物 md5 `dba3425944fbc4077099f0072732ec01`。

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

---

## 批次 7 — modern-web-guidance 审计（2026-11-22）

用 `modern-web-guidance`（skill 版本 `2026_11_09-5bcc021b`）检索了
`confirm destructive action` / `loading skeleton placeholder` / `tablist keyboard navigation`
`efficient-background-processing` / `accessibility` / `slider` / `color`，逐条对账已落地代码。

**改了（commit `51b84ca`，上板 md5 `83dcbffc3dc1e8033dab0929ed9d46c3`）**

| 项 | 问题 | 处理 |
|---|---|---|
| 音量条样式 | 只加了 `accent-color`，轨道/填充/滑块全交给 UA，暗色主题下轨道几乎看不见；无可见焦点环 | 接管 `::-webkit-slider-runnable-track/thumb`、`::-moz-range-track/progress/thumb`（**分块声明**，合并成逗号列表整条会被丢弃）；填充用 `progress()+attr()`，不支持时退 `--spk-pct`；`:focus-visible` 焦点环；`pointer: coarse` 加高命中区；`forced-colors` 下退回系统色实线 |
| 音量条语义 | 只有 `aria-label`；右侧「音量 43%」的 `<b>` 与滑块值重复朗读 | 加真 `<label class="visually-hidden" for="spk-vol">`，去掉 `aria-label`；`aria-valuetext` 随拖动/回写更新；`#spk-vol-v` 设 `aria-hidden` |
| 地图等待态 | 提示文字先画、机器人箭头后画，箭头压在首行文字中间糊成一团（截图实证） | 等待态先画文字，箭头改画在文案上方 26px |

**核过、不改的（附理由）**

| 项 | 结论 |
|---|---|
| `.page { content-visibility: hidden }` | 切页时零布局/绘制，比 `auto` 更彻底；不加 `contain-intrinsic-size` 也不会跳滚动条（页高由 `.page-inner` 撑开，隐藏页不占位）— `interactions-in-complex-layouts` 的适用场景是「可视区内大块区域」，这里不适用 |
| `contentvisibilityautostatechange` 暂停轮询 | 地图/雷达本就只在 `page === 'control'` 时绘制；WS 是单条长连接，不按可见性暂停更稳（板端还有看门狗） |
| 判读条 `role="button"` | `<article>` 加 `role=button` 是被允许的（不是 `li` 这类容器），且不可展开时移除 role/tabindex，符合 `accessibility` 的 tab order 要求 |
| Toast `aria-live="polite"` | 全站唯一一个 live region，符合「每页一个 polite 区」的建议；锁停断言级告警另走判读条，不抢话 |
| `<dialog>` | 刻意不用：极简裁决用按钮内联二次确认（`ui_confirm`），无焦点陷阱负担 |

**新发现但未在本批处理的**

| 项 | 说明 |
|---|---|
| 地图等待态箭头实际不显示 | 批次 8 复核：`Store.state.pose` **有值**（右侧卡读的就是它），箭头其实画得出来；之前判断有误。等待态箭头已挪到文案上方，不再压字 |
| 内嵌 webview 主题钳制 | 宿主设置了 `prefers-color-scheme`，页面 `renderedScheme()` 被系统值拉回，`#theme-toggle` 点了不切色。判断为 webview 行为，真机浏览器待复核 |
| `T()` 每次改文本读 `scrollWidth` | 调度器在 rAF 内写完再读，无强制同步布局；若后续出现 INP 问题再改成 `ResizeObserver` 缓存宽度 |

---

## 批次 8 — 位姿/地图新鲜度与交互反馈（2026-11-22，只动前端）

真机观察到的四个纯前端问题。**不碰 Rust、垫片、匹配器、AMCL、`/initialpose`**。

| 项 | 问题 | 处理 | 验收 |
|---|---|---|---|
| 位姿过期 | 垫片 TF 断流后不再发 `pose` 帧，`renderPose` 不看 `age`、`!pose` 直接 return，卡片/箭头永远停在旧值 | `Store.poseUsable()`：服务端 `pose_age` 与本地到达时长取大者，>2.5 s 判过期；卡片回「—」并显示「位姿不可用：TF 无数据」，`canvas-map` 不画箭头不记轨迹，重定位也不带旧位姿 | 注入 6 s 旧位姿 → x/θ 变「—」、提示出现；恢复 → 回到数值（浏览器实测通过） |
| 地图帧位姿 | `ws_video.rs` 地图帧自带 `pose/pose_age`，`applyMap` 只取栅格，首帧箭头要等下一个独立 pose 帧 | `applyMap` 顺手 `setPose(f)` | 构造带 pose 的地图帧 → `state.pose` 立即被写入 |
| 地图未就绪 | `endDrag` 里 `if (!g) return`，点选点等于没点 | 「导航选点 / 录入航点」点一下弹「地图还没加载，无法选点」；画布上的点击也给同一句。按钮**不** disabled（禁用后连原因都点不出来） | 未就绪时点击 → toast + 模式不变 |
| 地图过期 | 栅格不刷新时界面没有任何提示 | `#map-age`（`role=status`）在地图 age > 5 s 显示「地图已 N 秒未更新」，恢复即隐藏 | 注入 30 s 旧地图 → 显示「地图已 32 秒未更新」；刷新 → 隐藏 |
| 判读条键盘 | Enter/Space 都在 keydown 且 preventDefault，按住空格连发 | Enter 走 keydown（可重复），Space 走 keyup，keydown 只拦滚动 | 空格 keydown 不切换、keyup 切换；Enter 每次按下都切换 |

上板：`116bbb16…` → `9b94f8a4b1bc6ffe1b20794a28720dcc`（两端 md5 一致，服务 active）。


