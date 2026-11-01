# 麦克纳姆轮运动学交互动画

与固件同源的逆运动学模型（`car.mjs` 的 `M`/`WHEELS`/`makeCar`）驱动的浏览器交互动画：等距视角场地 + 麦轮车 + 控制台，实时展示 V±T 混合下四轮转速与方向。

## 直接看效果（任选其一，双击本地打开或部署到任意静态托管）

- [mecanum.html](mecanum.html) — 暗色主题
- [mecanum-light.html](mecanum-light.html) — 浅色主题

交互：拖动摇杆 / 扭环、预设动作按钮、方向键 + QE（前进/平移/自转）。

## 源码

| 文件 | 作用 |
| --- | --- |
| `build.mjs` | 构建入口：场地、控制台、审计、生成页面 |
| `car.mjs` | 车体几何 + 逆运动学模型（页面与构建双端共用） |
| `console.mjs` | 控制台交互件（摇杆/按钮/轮速条） |
| `live.js` | 页面运行时（物理积分、输入、游走演示） |

重新生成页面：`node build.mjs`（暗色）/ `node build.mjs --light`；几何审计：`node build.mjs --audit`。

> 注意：`build.mjs` 引用了本机的绝对路径工具链（`/home/snow/.agents/skills/anatomy/kit/`），在其他机器上重建需先调整这些 import 到 kit 的实际位置。仓库里的两份 HTML 是自包含产物，可直接使用、不依赖工具链。

`shots/` 为各主题与特写的验证截图。

## 与固件的关系

运动学口径与固件 `Core/Src/drive_ctrl.c` 的混合公式一致：A(右前)=V+T、B(左前)=V−T、C(右后)=V+T、D(左后)=V−T，峰值为四轮绝对值最大者、超限等比缩放。轮序/符号表见仓库根 README 的"轮序与驱动"表。
