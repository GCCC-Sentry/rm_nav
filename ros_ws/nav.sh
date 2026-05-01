#!/bin/bash
# 进入ROS2工作空间
cd ~/ros_ws

# 屏蔽 accessibility bus 警告（Docker 环境无此服务）
export NO_AT_BRIDGE=1
ROS_ENV_CMD="source /opt/ros/humble/setup.bash && source install/setup.bash"

# 确保 dbus 可用（Docker 环境需要）
if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    eval "$(dbus-launch --sh-syntax)"
fi

# 使用 gnome-terminal 启动各节点
gnome-terminal --title="rm_static_tf" -- bash -c "$ROS_ENV_CMD && ros2 launch rm_static_tf static_tf.launch.py; exec bash" &

gnome-terminal --title="adam_map2odom" -- bash -c "$ROS_ENV_CMD && ros2 launch adam_map2odom map2odom.launch.py; exec bash" &

sleep 2

gnome-terminal --title="pb2025_nav_bringup" -- bash -c "$ROS_ENV_CMD && ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=bl slam:=False use_robot_state_pub:=False; exec bash" &

sleep 3

gnome-terminal --title="my_serial_py" -- bash -c "$ROS_ENV_CMD && ros2 launch my_serial_py serial.launch.py; exec bash" &

sleep 6

gnome-terminal --title="pb2025_sentry_behavior" -- bash -c "$ROS_ENV_CMD && ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py; exec bash" &

sleep 2

# 启动姿态监视器（独立终端显示）
# gnome-terminal --title="Stance Monitor (姿态监视器)" -- bash -c "source install/setup.bash && python3 install/pb2025_sentry_behavior/share/pb2025_sentry_behavior/scripts/stance_monitor.py; exec bash" &


tail -f /dev/null
