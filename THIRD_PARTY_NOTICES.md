# 第三方组件许可

本仓库引用的下列组件各自受其原有许可约束，不受根目录 `LICENSE`（MIT）覆盖。

| 组件 | 位置 | 许可 |
|---|---|---|
| STM32F4 HAL 驱动与 CMSIS | 各车型分支 `Drivers/` | ST 公司 BSD-3-Clause |
| FreeRTOS Kernel | 各车型分支 `Middlewares/Third_Party/FreeRTOS` | MIT |
| STM32 USB Host Library | 各车型分支 `Middlewares/ST/STM32_USB_Host_Library` | ST 公司 SLA0044 |

## 说明

- MIT 只覆盖本仓库作者编写的代码与文档。
- 上述组件的原始版权声明保留在各自源文件中，请勿删除。
- 分发固件时需同时遵守这些组件的许可条款。
- SLA0044 对再分发的限制比 MIT 严格，商用前请自行核对原文。
