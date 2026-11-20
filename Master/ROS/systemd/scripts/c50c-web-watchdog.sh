#!/bin/bash
# web_debug 看门狗：c50c-web-watchdog.timer 每分钟调一次。
# 8088 无应答就重启 c50c-web.service（服务自身 Restart=always 兜进程崩溃，
# 这里兜"进程活着但端口不服务"的僵死）。不清 fastrtps SHM——底盘桥还活着。
if curl -s --max-time 4 http://127.0.0.1:8088/api/state | grep -q 'battery'; then
    exit 0
fi
echo "$(date '+%F %T') watchdog: 8088 无应答，重启 c50c-web" >> /tmp/wb.log
systemctl --user restart c50c-web.service
