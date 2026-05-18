#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_FILE="$REPO_DIR/match_nav_config.env"

DEFAULT_CONTAINER="ros_nav_stable_final"
DEFAULT_NAMESPACE="red_standard_robot1"
DEFAULT_WORLD="map"
DEFAULT_PCD="$REPO_DIR/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/point_lio/PCD/scans.pcd"

MAP_DIR_HOST="$REPO_DIR/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality"
PCD_DIR_HOST="$REPO_DIR/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality"
MAP_DIR_CONTAINER="/root/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality"
ROS_ENV_CMD="source /opt/ros/humble/setup.bash && source /root/ros_ws/install/setup.bash"

WORLD=""
CONTAINER=""
NAMESPACE=""
PCD_PATH=""
SKIP_PCD="0"
WORLD_SET_BY_ARG="0"

usage() {
    cat <<'EOF'
用法:
  ./scripts/save_current_map.sh [--world <地图名>] [--namespace <命名空间>] [--container <容器名>] [--pcd <pcd路径>] [--skip-pcd]

说明:
  1. 直接从正在运行的建图容器里执行 map_saver_cli
  2. 栅格图直接保存到 pb2025_nav_bringup/map/reality/<world>.yaml/.pgm
  3. 默认同时复制 point_lio 的 scans.pcd 到 pcd/reality/<world>.pcd
  4. 自动把 match_nav_config.env 里的 WORLD 更新为当前地图名
EOF
}

if [ -f "$CONFIG_FILE" ]; then
    # shellcheck disable=SC1090
    source "$CONFIG_FILE"
fi

CONTAINER="${CONTAINER:-$DEFAULT_CONTAINER}"
NAMESPACE="${ROBOT_NAMESPACE:-$DEFAULT_NAMESPACE}"
PCD_PATH="${PCD_PATH:-$DEFAULT_PCD}"

while [ $# -gt 0 ]; do
    case "$1" in
        --world)
            WORLD="${2:-}"
            WORLD_SET_BY_ARG="1"
            shift 2
            ;;
        --namespace)
            NAMESPACE="${2:-}"
            shift 2
            ;;
        --container)
            CONTAINER="${2:-}"
            shift 2
            ;;
        --pcd)
            PCD_PATH="${2:-}"
            shift 2
            ;;
        --skip-pcd)
            SKIP_PCD="1"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "未知参数: $1" >&2
            usage
            exit 1
            ;;
    esac
done

if [ "$WORLD_SET_BY_ARG" != "1" ]; then
    CURRENT_WORLD="${WORLD:-$DEFAULT_WORLD}"
    read -r -p "请输入地图名 [${CURRENT_WORLD}]: " INPUT_WORLD
    WORLD="${INPUT_WORLD:-$CURRENT_WORLD}"
fi

if [ -z "$WORLD" ]; then
    echo "错误: 地图名不能为空" >&2
    exit 1
fi

if ! [[ "$WORLD" =~ ^[A-Za-z0-9_-]+$ ]]; then
    echo "错误: 地图名仅允许字母、数字、下划线和中划线: $WORLD" >&2
    exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
    echo "错误: 未检测到 docker" >&2
    exit 1
fi

if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
    echo "错误: 容器 $CONTAINER 不存在" >&2
    exit 1
fi

if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER")" != "true" ]; then
    echo "错误: 容器 $CONTAINER 未运行，请先启动建图环境" >&2
    exit 1
fi

mkdir -p "$MAP_DIR_HOST" "$PCD_DIR_HOST"

TARGET_YAML_HOST="$MAP_DIR_HOST/$WORLD.yaml"
TARGET_PGM_HOST="$MAP_DIR_HOST/$WORLD.pgm"
TARGET_PREFIX_CONTAINER="$MAP_DIR_CONTAINER/$WORLD"

echo "开始保存当前地图..."
echo "  地图名: $WORLD"
echo "  容器: $CONTAINER"
echo "  命名空间: /$NAMESPACE"

docker exec -e ROS_DISTRO=humble -it "$CONTAINER" bash -lc "
    set -euo pipefail
    mkdir -p '$MAP_DIR_CONTAINER'
    cd /root/ros_ws
    $ROS_ENV_CMD
    ros2 run nav2_map_server map_saver_cli -f '$TARGET_PREFIX_CONTAINER' --ros-args -r __ns:=/$NAMESPACE
"

if [ ! -f "$TARGET_YAML_HOST" ] || [ ! -f "$TARGET_PGM_HOST" ]; then
    echo "错误: map_saver_cli 执行后未找到输出文件" >&2
    echo "  期望 YAML: $TARGET_YAML_HOST" >&2
    echo "  期望 PGM:  $TARGET_PGM_HOST" >&2
    exit 1
fi

if [ "$SKIP_PCD" != "1" ]; then
    if [ -f "$PCD_PATH" ]; then
        cp "$PCD_PATH" "$PCD_DIR_HOST/$WORLD.pcd"
    else
        echo "警告: 未找到 PCD 文件 $PCD_PATH，已跳过 PCD 复制" >&2
    fi
fi

if [ -f "$CONFIG_FILE" ]; then
    TMP_CONFIG="$(mktemp)"
    awk -v world="$WORLD" '
        BEGIN { updated = 0 }
        /^WORLD=/ {
            print "WORLD=\"" world "\""
            updated = 1
            next
        }
        { print }
        END {
            if (!updated) {
                print "WORLD=\"" world "\""
            }
        }
    ' "$CONFIG_FILE" > "$TMP_CONFIG"
    mv "$TMP_CONFIG" "$CONFIG_FILE"
fi

echo
echo "地图保存完成:"
echo "  YAML: $TARGET_YAML_HOST"
echo "  PGM:  $TARGET_PGM_HOST"
if [ "$SKIP_PCD" != "1" ] && [ -f "$PCD_DIR_HOST/$WORLD.pcd" ]; then
    echo "  PCD:  $PCD_DIR_HOST/$WORLD.pcd"
fi
echo
echo "已自动更新:"
echo "  $CONFIG_FILE -> WORLD=\"$WORLD\""
echo
echo "下一步:"
echo "  1. 结束建图后启动 ./start_nav_host.sh"
echo "  2. 不需要为这张地图重新编译"
