#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${CONTAINER:-ros_nav_stable_final}"
RECREATE_SCRIPT="$SCRIPT_DIR/recreate_ros_nav_container.sh"
ROS_ENV_CMD="source /opt/ros/humble/setup.bash && source /root/ros_ws/install/setup.bash"

if ! command -v docker >/dev/null 2>&1; then
    echo "错误：未检测到 docker"
    exit 1
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "错误：未检测到 gnome-terminal，无法可视化启动"
    exit 1
fi

if [ ! -x "$RECREATE_SCRIPT" ]; then
    echo "错误：未找到可执行脚本 $RECREATE_SCRIPT"
    exit 1
fi

if [ -z "${DISPLAY:-}" ]; then
    export DISPLAY=:0
fi

# 允许容器内的 GUI 程序连接宿主机 X11。
xhost +SI:localuser:root >/dev/null 2>&1 || true
xhost +local:docker >/dev/null 2>&1 || true

if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
    echo "容器 $CONTAINER 不存在，开始自动创建..."
    "$RECREATE_SCRIPT"
fi

if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER")" != "true" ]; then
    echo "容器 $CONTAINER 未运行，正在启动..."
    if ! docker start "$CONTAINER" >/dev/null; then
        echo "检测到旧容器启动失败，正在重建容器..."
        "$RECREATE_SCRIPT"
    fi
fi

WORKSPACE_PROBE="/root/ros_ws/install/pb2025_nav_bringup/share/pb2025_nav_bringup/local_setup.bash"

if ! docker exec "$CONTAINER" bash -lc "
    if [ ! -L '$WORKSPACE_PROBE' ]; then
        exit 1
    fi
    target=\$(readlink '$WORKSPACE_PROBE')
    case \"\$target\" in
        /root/ros_ws/*) exit 0 ;;
        *) exit 1 ;;
    esac
"; then
    echo "检测到容器内 ROS 工作区 install 链接无效，正在容器内重建..."
    docker exec -e ROS_DISTRO=humble -it "$CONTAINER" bash -lc \
        "cd /root/ros_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install"
fi

echo "正在启动 ROS2 导航系统..."
echo "将打开 5 个独立终端窗口"

gnome-terminal --title="rm_static_tf" -- bash -c "docker exec -e ROS_DISTRO=humble -it $CONTAINER bash -lc 'cd /root/ros_ws && $ROS_ENV_CMD && ros2 launch rm_static_tf static_tf.launch.py'; exec bash" &
sleep 0.5

gnome-terminal --title="adam_map2odom" -- bash -c "docker exec -e ROS_DISTRO=humble -it $CONTAINER bash -lc 'cd /root/ros_ws && $ROS_ENV_CMD && ros2 launch adam_map2odom map2odom.launch.py'; exec bash" &
sleep 2

gnome-terminal --title="pb2025_nav_bringup" -- bash -c "docker exec -e ROS_DISTRO=humble -it $CONTAINER bash -lc 'cd /root/ros_ws && $ROS_ENV_CMD && ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=qian1 slam:=False use_robot_state_pub:=False'; exec bash" &
sleep 3

gnome-terminal --title="my_serial_py" -- bash -c "docker exec -e ROS_DISTRO=humble -it $CONTAINER bash -lc 'cd /root/ros_ws && $ROS_ENV_CMD && ros2 launch my_serial_py serial.launch.py'; exec bash" &
sleep 3

gnome-terminal --title="pb2025_alliance_decision" -- bash -c "docker exec -e ROS_DISTRO=humble -it $CONTAINER bash -lc 'cd /root/ros_ws && $ROS_ENV_CMD && ros2 launch pb2025_alliance_decision alliance_decision.launch.py'; exec bash" &

echo "======================================"
echo "所有终端窗口已启动"
echo "容器名称: $CONTAINER"
echo "停止方法:"
echo "  1. 在每个终端窗口中按 Ctrl+C"
echo "  2. 如需停容器，执行: docker stop $CONTAINER"
echo "======================================"
