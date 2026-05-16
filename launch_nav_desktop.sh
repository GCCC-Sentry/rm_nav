#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "错误：未检测到 gnome-terminal，无法从桌面启动导航"
    exit 1
fi

gnome-terminal --title="导航启动器" -- bash -lc "
    cd '$SCRIPT_DIR'
    ./start_nav_host.sh
    status=\$?
    echo
    if [ \$status -eq 0 ]; then
        echo '导航启动命令已执行，终端窗口可直接关闭。'
    else
        echo \"导航启动失败，退出码: \$status\"
    fi
    echo '按任意键关闭此窗口...'
    read -n 1 -s -r
    exit \$status
"
