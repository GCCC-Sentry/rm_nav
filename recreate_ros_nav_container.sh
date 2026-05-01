#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${IMAGE:-kswlt/nav_20260226_yujiantu_gicp_hebavior_final_release_version1:latest}"
CONTAINER="${CONTAINER:-ros_nav_stable_final}"
ROS_WS_PATH="${ROS_WS_PATH:-$SCRIPT_DIR/ros_ws}"
DISPLAY_VALUE="${DISPLAY:-:0}"
XSOCK="/tmp/.X11-unix"
XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"
XAUTH_STABLE="${XAUTH_STABLE:-$HOME/.docker.xauth}"

if [ ! -d "$ROS_WS_PATH" ]; then
    echo "错误：未找到 ros_ws 目录: $ROS_WS_PATH"
    exit 1
fi

if [ ! -d "$XSOCK" ]; then
    echo "错误：未找到 X11 socket 目录: $XSOCK"
    exit 1
fi

# Wayland / Xwayland 场景下，XAUTHORITY 常常指向 /run/user/.../.mutter-Xwaylandauth.*
# 这类文件会在重启或重新登录后变化，直接 bind mount 会导致容器下次自动重启时挂载失效。
# 这里统一复制到一个稳定路径，再挂进容器。
if [ -f "$XAUTH" ]; then
    cp "$XAUTH" "$XAUTH_STABLE"
    chmod 600 "$XAUTH_STABLE"
fi

echo "拉取镜像: $IMAGE"
docker pull "$IMAGE"

if docker inspect "$CONTAINER" >/dev/null 2>&1; then
    echo "删除旧容器: $CONTAINER"
    docker rm -f "$CONTAINER" >/dev/null
fi

RUN_ARGS=(
    -d
    --name "$CONTAINER"
    --restart always
    --network host
    --privileged
    -e "DISPLAY=$DISPLAY_VALUE"
    -e "ROS_DISTRO=humble"
    -v "$ROS_WS_PATH:/root/ros_ws"
    -v "/dev:/dev"
    -v "$XSOCK:$XSOCK"
)

if [ -f "$XAUTH_STABLE" ]; then
    RUN_ARGS+=(-e "XAUTHORITY=/root/.Xauthority" -v "$XAUTH_STABLE:/root/.Xauthority:ro")
fi

echo "创建新容器: $CONTAINER"
docker run "${RUN_ARGS[@]}" "$IMAGE" tail -f /dev/null

echo "容器已重建完成"
echo "镜像: $IMAGE"
echo "工作区挂载: $ROS_WS_PATH -> /root/ros_ws"
echo "网络模式: host"
echo "USB/设备权限: /dev 全挂载 + --privileged"
echo "自启动策略: always"
