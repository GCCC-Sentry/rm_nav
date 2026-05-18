#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "错误：未检测到 gnome-terminal，无法从桌面启动地图保存器"
    exit 1
fi

gnome-terminal --title="保存当前地图" -- bash -lc "
    cd '$SCRIPT_DIR'
    ./scripts/save_current_map.sh
    status=\$?
    echo
    if [ \$status -eq 0 ]; then
        echo '地图已保存并部署完成。'
    else
        echo \"地图保存失败，退出码: \$status\"
    fi
    echo '按任意键关闭此窗口...'
    read -n 1 -s -r
    exit \$status
"
