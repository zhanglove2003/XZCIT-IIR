## 改了什么

<!-- 一句话说清这个 PR 做的事 -->

## 为什么改

<!-- 关联议题编号，例如 Closes #12；或说明动机 -->

## 验证方式

<!-- 勾选实际做过的，没做的不要勾 -->

- [ ] 本地 `make -j8` 编译通过，零告警
- [ ] `bash tools/test_motion.sh` 通过
- [ ] `bash tools/test_boot_ctrl_integration.sh` 通过
- [ ] `python3 tools/test_font_convert.py` 通过
- [ ] 实车验证（说明验证了哪些场景）
- [ ] 只改文档，无需上述验证

## 影响范围

<!-- 涉及哪个车型？改了哪些模块？会不会影响其他配置？ -->

- 车型：<!-- 麦轮 / 差分 / 通用 -->
- 模块：<!-- 例如 drive_ctrl、oled_gfx、自检服务 -->
- 硬件影响：<!-- 是否改引脚、改外设配置；改了就更新 knowledge/pinmap.md -->

## 合并前检查

- [ ] 分支已与目标分支同步
- [ ] CI 全绿（`ci-success`）
- [ ] 改了引脚或外设，`knowledge/pinmap.md` 已同步更新
- [ ] 改了分支模型或 CI 规则，`docs/workflow.md` 已同步更新
