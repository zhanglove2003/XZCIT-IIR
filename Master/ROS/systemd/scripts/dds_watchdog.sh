#!/bin/bash
# dds_watchdog.sh —— T2 DDS 看门狗 v2（执行计划书 v4 20261116）
# 探测两步走：先 topic info 查发布者（区分硬件故障与脑裂），再 echo --once。
# 无 set -e；不设置 DDS domain 变量（见执行计划书 C1）；无工作站路径回退；冷却 5 分钟；
# 连续 3 次恢复失败 → 只告警停止自动恢复，等人工介入。

LOG_TAG="dds-watchdog"       # journalctl 标识（systemd unit SyslogIdentifier 同名）
START_GRACE_SEC=120          # 启动宽限，避开建图初始化窗口
PROBE_INTERVAL_SEC=30        # 主循环探测间隔
COOLDOWN_SEC=300             # 恢复后冷却 5 分钟
MAX_RECOVER_FAILS=3          # 连续恢复失败熔断阈值
RECOVER_WAIT_SEC=20          # 恢复后等栈起来的时间
RADAR_PORT=""                # [N2] 巡检兜底拉起 lidar_node 所需串口，recover 内动态探测

log() { echo "[$LOG_TAG][$(date '+%F %T')] $*"; }

# 等栈起来
wait_stack_settle() {
    sleep "$RECOVER_WAIT_SEC"
    return 0
}

# 探测：返回 0=健康  1=脑裂(有发布者但收不到数据)  2=无发布者或环境异常(只告警,不动手)
# 计划书 T2 判据：只有「有发布者但 echo 收不到数据」才授权进入恢复流程；
# topic info 为空 / ros2 环境损坏 / 发布者为 0，均按 T2 只告警，绝不 pkill 重启栈。
probe() {
    # ros2 环境不可用：只告警，不进入恢复流程
    if ! command -v ros2 >/dev/null 2>&1; then
        return 2
    fi

    local info_out
    info_out=$(timeout 5 ros2 topic info /scan 2>/dev/null)
    # info 输出为空（DDS 未就绪/环境异常/瞬时抖动）：只告警，不动栈
    if [ -z "$info_out" ]; then
        return 2
    fi

    if echo "$info_out" | grep -q "Publisher count: [1-9]"; then
        :  # 有发布者，继续步骤2
    else
        # 有 info 输出但发布者数为 0 → 雷达硬件故障/拔线，只告警不动手
        return 2
    fi

    # echo 的 Python 输出需要无缓冲，避免已收到消息却在超时前未写出。
    local scan_out
    scan_out=$(PYTHONUNBUFFERED=1 timeout 10 ros2 topic echo --once /scan \
        --qos-reliability best_effort \
        --qos-durability volatile 2>/dev/null)
    if printf '%s\n' "$scan_out" | grep -q '^header:$'; then
        return 0
    fi
    return 1
}

# 恢复流程：精确 pkill + 重调 start_web.sh
recover() {
    log "开始恢复流程：停止部分 ROS 进程、重启 start_web.sh；保留共享内存"

    # 精确 pkill：只匹配本栈进程（start_web.sh 全路径锚定；节点按 install 库路径可执行文件名匹配，
    # 避免任何命令行里恰好含 ros2_ws / 节点名的无关进程被误杀）
    pkill -f '/home/orangepi/start_web.sh' || true
    pkill -f '/home/orangepi/ros2_ws/install/opi_robot/lib/opi_robot/camera_node' || true
    pkill -f '/home/orangepi/ros2_ws/install/opi_robot/lib/opi_robot/lidar_node' || true
    pkill -f '/home/orangepi/ros2_ws/install/opi_robot/lib/opi_robot/fusion_node' || true
    pkill -f '/home/orangepi/ros2_ws/install/opi_robot/lib/opi_robot/static_tf' || true
    pkill -f '/home/orangepi/ros2_ws/install/opi_robot/lib/opi_robot/web_debug' || true
    pkill -f '/home/orangepi/ros2_ws/install/c50c_uart3/lib/c50c_uart3/c50c_uart3_node' || true
    pkill -f '/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node' || true
    sleep 3

    # EKF、ROS daemon 等进程可能仍在使用共享内存，不能在此删除。

    # 重启全栈
    cd /home/orangepi || true
    nohup bash /home/orangepi/start_web.sh >> /home/orangepi/.cache/opi-robot/dds-watchdog-recover.log 2>&1 || true

    wait_stack_settle

    # 恢复后确保底盘桥在位：start_web.sh 不含桥（systemd 服务），11-17 两次恢复均把桥杀在 dead
    systemctl --user reset-failed c50c-chassis 2>/dev/null || true
    systemctl --user start c50c-chassis || true
    sleep 3
    if systemctl --user is-active --quiet c50c-chassis; then
        log "底盘桥 c50c-chassis 已恢复 active"
    else
        log "底盘桥 c50c-chassis 启动失败，需人工介入"
    fi

    # ---- [N2 2026-11-17] 恢复流程巡检兜底：三节点不在的按入口脚本环境拉起，已存活则跳过 ----
    # start_web.sh 已幂等化，但保险起见此处独立巡检一次；绝不 pkill 正常节点。
    local LIB=/home/orangepi/ros2_ws/install/opi_robot/lib/opi_robot
    local pat name cmd
    # [N2] 动态探测雷达串口（同 start_web.sh 的 by-id 优先逻辑）
    RADAR_PORT=""
    for p in /dev/serial/by-id/*CP2102*-if00-port0; do
        [ -e "$p" ] && RADAR_PORT="$p" && break
    done
    [ -z "$RADAR_PORT" ] && RADAR_PORT=$(ls /dev/ttyUSB* 2>/dev/null | head -1)
    for pair in "camera_node:python3 $LIB/camera_node --ros-args -p width:=640 -p height:=480" \
                "lidar_node:python3 $LIB/lidar_node --ros-args -p lidar_port:=$RADAR_PORT" \
                "fusion_node:python3 $LIB/fusion_node --ros-args -p enable_cmd_vel:=true -p cruise_max_run_s:=1.0 -p body_half_length:=0.2 -p body_half_width:=0.2 -p body_center_x:=-0.08"; do
        pat="${pair%%:*}"
        cmd="${pair#*:}"
        if pgrep -f "$LIB/$pat" >/dev/null 2>&1; then
            log "巡检：$pat 已存活，跳过"
        else
            log "巡检：$pat 不在，按入口脚本环境拉起"
            # shellcheck disable=SC2086
            setsid nohup $cmd > "/tmp/${pat%.py}.log" 2>&1 < /dev/null &
        fi
    done
}

# ---- 主流程 ----
mkdir -p /home/orangepi/.cache/opi-robot || true

log "DDS 看门狗启动，宽限 ${START_GRACE_SEC}s 后开始探测"
sleep "$START_GRACE_SEC"

fail_count=0          # 连续恢复失败计数
in_cooldown_until=0   # 冷却截止 epoch

while true; do
    now=$(date +%s)

    # 冷却窗口内不探测
    if [ "$now" -lt "$in_cooldown_until" ]; then
        sleep 15
        continue
    fi

    # 确保 ros2 环境可用
    if ! command -v ros2 >/dev/null 2>&1; then
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash || true
        # shellcheck disable=SC1091
        source /home/orangepi/ros2_ws/install/setup.bash || true
    fi

    probe
    rc=$?

    if [ "$rc" -eq 0 ]; then
        # 熔断计数清零需人工 touch /tmp/dds-watchdog-reset，防止熔断被自动复融
        if [ "$fail_count" -ge "$MAX_RECOVER_FAILS" ]; then
            if [ -f /tmp/dds-watchdog-reset ]; then
                rm -f /tmp/dds-watchdog-reset
                fail_count=0
                log "人工复位生效：/tmp/dds-watchdog-reset 已消费，fail_count 清零"
            else
                log "健康检测：/scan 正常，但已熔断（fail_count=$fail_count），保持熔断——人工 touch /tmp/dds-watchdog-reset 才能复位"
            fi
        else
            log "健康检测：/scan 正常（发布者存在且 echo 收到数据）"
            fail_count=0
        fi
        sleep "$PROBE_INTERVAL_SEC"
        continue
    fi

    if [ "$rc" -eq 2 ]; then
        log "告警：/scan 无发布者或环境异常——按 T2 只告警不动栈（不清熔断计数 fail_count=$fail_count）"
        # 注意：这里不清零 fail_count——硬件故障/环境异常与脑裂熔断计数无关，
        # 避免瞬时无发布者穿插清零，破坏「连续3次恢复失败→熔断」逻辑。
        sleep "$PROBE_INTERVAL_SEC"
        continue
    fi

    # rc=1：脑裂
    if [ "$fail_count" -ge "$MAX_RECOVER_FAILS" ]; then
        # 熔断后进入长冷却，告警降频（每 COOLDOWN_SEC 最多一条），避免刷屏 journal
        in_cooldown_until=$(( $(date +%s) + COOLDOWN_SEC ))
        log "告警：脑裂持续，已连续 ${fail_count} 次恢复失败，熔断——停止自动恢复，人工 touch /tmp/dds-watchdog-reset 才能复位"
        sleep "$PROBE_INTERVAL_SEC"
        continue
    fi

    log "检测到脑裂：有发布者但 echo --once 收不到 /scan（第 $((fail_count+1)) 次恢复尝试）"
    recover

    # 复检
    probe
    rc2=$?
    if [ "$rc2" -eq 0 ]; then
        log "恢复成功：/scan 数据恢复"
        if [ "$fail_count" -ge "$MAX_RECOVER_FAILS" ]; then
            if [ -f /tmp/dds-watchdog-reset ]; then
                rm -f /tmp/dds-watchdog-reset
                fail_count=0
                log "人工复位生效：/tmp/dds-watchdog-reset 已消费，fail_count 清零"
            else
                log "恢复成功但已熔断（fail_count=$fail_count），保持熔断——人工 touch /tmp/dds-watchdog-reset 才能复位"
            fi
        else
            fail_count=0
        fi
        in_cooldown_until=$(( $(date +%s) + COOLDOWN_SEC ))
        log "进入冷却 ${COOLDOWN_SEC}s"
    else
        fail_count=$((fail_count+1))
        log "恢复后复检仍不健康（rc=$rc2），失败计数 fail_count=$fail_count"
        if [ "$fail_count" -ge "$MAX_RECOVER_FAILS" ]; then
            log "告警：连续 ${fail_count} 次恢复失败，熔断——停止自动恢复，人工 touch /tmp/dds-watchdog-reset 才能复位"
            # 熔断后进入长冷却，告警降频（每 5 分钟最多一条），避免刷屏 journal
            in_cooldown_until=$(( $(date +%s) + COOLDOWN_SEC ))
        fi
        # 恢复失败不进冷却：保持 30s 探测节奏，尽快再次尝试恢复
    fi
done
