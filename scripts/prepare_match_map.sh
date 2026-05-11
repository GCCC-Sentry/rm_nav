#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BRINGUP_DIR="$REPO_DIR/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup"
MAP_DIR="$BRINGUP_DIR/map/reality"
PCD_DIR="$BRINGUP_DIR/pcd/reality"
DEFAULT_PCD="$REPO_DIR/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/point_lio/PCD/scans.pcd"

usage() {
    cat <<'EOF'
用法:
  ./scripts/prepare_match_map.sh --map-yaml <保存出的yaml路径> --world <地图名> [--pcd <pcd路径>] [--skip-pcd]

示例:
  ./scripts/prepare_match_map.sh --map-yaml /home/asus/tmp/qian1.yaml --world qian1
  ./scripts/prepare_match_map.sh --map-yaml ./three.yaml --world qian1 --pcd ./scans.pcd
EOF
}

MAP_YAML=""
WORLD=""
PCD_PATH="$DEFAULT_PCD"
SKIP_PCD="0"

while [ $# -gt 0 ]; do
    case "$1" in
        --map-yaml)
            MAP_YAML="${2:-}"
            shift 2
            ;;
        --world)
            WORLD="${2:-}"
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

if [ -z "$MAP_YAML" ] || [ -z "$WORLD" ]; then
    echo "错误: --map-yaml 和 --world 必填" >&2
    usage
    exit 1
fi

if [ ! -f "$MAP_YAML" ]; then
    echo "错误: 找不到地图 YAML: $MAP_YAML" >&2
    exit 1
fi

mkdir -p "$MAP_DIR" "$PCD_DIR"

MAP_YAML_ABS="$(readlink -f "$MAP_YAML")"
MAP_SOURCE_DIR="$(dirname "$MAP_YAML_ABS")"
IMAGE_RELATIVE="$(sed -n 's/^image:[[:space:]]*//p' "$MAP_YAML_ABS" | head -n 1)"

if [ -z "$IMAGE_RELATIVE" ]; then
    echo "错误: $MAP_YAML_ABS 中未找到 image: 字段" >&2
    exit 1
fi

case "$IMAGE_RELATIVE" in
    /*) IMAGE_SOURCE="$IMAGE_RELATIVE" ;;
    *) IMAGE_SOURCE="$MAP_SOURCE_DIR/$IMAGE_RELATIVE" ;;
esac

if [ ! -f "$IMAGE_SOURCE" ]; then
    echo "错误: 找不到 YAML 引用的图片文件: $IMAGE_SOURCE" >&2
    exit 1
fi

TARGET_YAML="$MAP_DIR/$WORLD.yaml"
TARGET_IMAGE_EXT="${IMAGE_SOURCE##*.}"
TARGET_IMAGE="$MAP_DIR/$WORLD.$TARGET_IMAGE_EXT"

cp "$IMAGE_SOURCE" "$TARGET_IMAGE"
sed "s|^image:.*$|image: $WORLD.$TARGET_IMAGE_EXT|" "$MAP_YAML_ABS" > "$TARGET_YAML"

if [ "$SKIP_PCD" != "1" ]; then
    if [ -f "$PCD_PATH" ]; then
        cp "$PCD_PATH" "$PCD_DIR/$WORLD.pcd"
    else
        echo "警告: 未找到 PCD 文件 $PCD_PATH，已跳过 PCD 复制" >&2
    fi
fi

echo "已更新比赛地图:"
echo "  YAML: $TARGET_YAML"
echo "  栅格图: $TARGET_IMAGE"
if [ "$SKIP_PCD" != "1" ] && [ -f "$PCD_DIR/$WORLD.pcd" ]; then
    echo "  PCD: $PCD_DIR/$WORLD.pcd"
fi
echo
echo "下一步建议:"
echo "  1. 把 match_nav_config.env 里的 WORLD 改成 $WORLD"
echo "  2. 运行 ./start_nav_host.sh"
echo "  3. 先用 RViz 手动点目标验证地图与点位"
