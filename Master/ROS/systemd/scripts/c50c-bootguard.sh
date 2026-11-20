#!/bin/bash
# 启动关键文件防清零守卫。
# 断电曾两次把 start_web.sh 数据块清零（2026-11-11、2026-11-15）：
# 元数据（大小/mtime）还在、内容全 NUL。开机对照金本副检查，
# 只在「丢失 / 全 NUL」这种数据丢失特征下还原，不覆盖正常改动。

set -u
GOLD=/home/orangepi/.opi-golden

declare -A LIVE=(
  ["start_web.sh"]="/home/orangepi/start_web.sh"
  ["start_chassis.sh"]="/home/orangepi/start_chassis.sh"
  ["c50c-web-watchdog.sh"]="/home/orangepi/c50c-web-watchdog.sh"
  ["c50c-web.service"]="/home/orangepi/.config/systemd/user/c50c-web.service"
  ["c50c-web-watchdog.timer"]="/home/orangepi/.config/systemd/user/c50c-web-watchdog.timer"
  ["c50c-web-watchdog.service"]="/home/orangepi/.config/systemd/user/c50c-web-watchdog.service"
)

for name in "${!LIVE[@]}"; do
  g="$GOLD/$name"; l="${LIVE[$name]}"
  if [ ! -f "$g" ]; then
    echo "bootguard: 金本缺失 $name，跳过"
    continue
  fi
  if [ ! -f "$l" ] || ! tr -d '\0' < "$l" 2>/dev/null | grep -q .; then
    cp "$g" "$l"
    echo "bootguard: 已还原 $name（丢失或被清零）"
  fi
done
echo "bootguard: 检查完成 $(date '+%F %T')"
