# STM32F407VE 标准库实车固件（StdPeriph，2026-11-20 实车烧录验证版）

本目录保留当前实际烧录在 C50C 小车 **STM32F407VE** 主控上的实车验证版本（基于标准外设库 StdPeriph + FreeRTOS）。

> 说明：仓库根目录（`App/`、`Core/`、`Drivers/`、`C50C_HAL.ioc` 等）保留正在推进的 CubeMX HAL 迁移代码；两者互不覆盖。

## 核心现场修复说明

- **`BALANCE/data_task.c` 编码器临界区快照修复（`enc_snap[4]`）**：
  - 修复 20 Hz `data_task` 在读取 4 路浮点轮速时跨越 100 Hz `balance_task` 更新周期导致的帧内撕裂，统一在同一临界区获取四轮速度快照，同时修正原厂按整型截断导致低速（<1 m/s）编码器反馈恒为 0 的问题（改为 `mm/s` 上报，确保底盘 `/odom_candidate` 精确闭环）。
- 构建说明见同目录 `Makefile` / `CMakeLists.txt` / `README_CMAKE.md`（构建中间产物 `build/`、`OBJ/`、`*.o`、`*.elf`、`*.bin` 已排除）。
