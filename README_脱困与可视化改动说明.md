# 脱困与可视化改动说明

## 1. 当前启动链路

当前主目录脚本 [start_nav_host.sh](/home/asus/nav3_mapping_nogicp_shortLA_stable/start_nav_host.sh:72) 实际启动的导航命令是：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=qian1 slam:=False use_robot_state_pub:=False
```

这条链路会加载实车参数文件 `config/reality/nav2_params.yaml`，并由 `bt_navigator` 默认加载：

- `navigate_to_pose_w_replanning_and_recovery.xml`
- `navigate_through_poses_w_replanning_and_recovery.xml`

只要上层发送的是 `navigate_to_pose` 或 `navigate_through_poses`，就会走这套恢复树。

## 2. 当前脱困逻辑是什么

当前不是“检测到卡住立刻脱困”，而是“导航失败后进入恢复树，再做清图和后退脱困”。

### 2.1 行为树恢复逻辑

`NavigateToPose` 恢复树位置：

- [navigate_to_pose_w_replanning_and_recovery.xml](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml:5)

`NavigateThroughPoses` 恢复树位置：

- [navigate_through_poses_w_replanning_and_recovery.xml](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml:5)

当前逻辑可以概括成：

1. 正常规划和跟踪
2. 规划失败时清全局代价地图
3. 跟踪失败时清局部代价地图
4. 外层恢复子树在“清图”和“BackUp”之间轮转
5. 反复尝试，不主动放弃

### 2.2 脱困后退逻辑

行为树里写的是：

```xml
<BackUp backup_dist="0.3" backup_speed="0.5"/>
```

但真正执行的不是 Nav2 默认 `BackUp`，而是 `behavior_server.backup` 绑定的自定义插件：

- [nav2_params.yaml](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml:612)

插件名：

- `pb_nav2_behaviors/BackUpFreeSpace`

插件实现位置：

- [back_up_free_space.cpp](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb_nav2_plugins/src/behaviors/back_up_free_space.cpp:54)
- [back_up_free_space.hpp](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb_nav2_plugins/include/pb_nav2_plugins/behaviors/back_up_free_space.hpp:34)

它的逻辑是：

1. 调用 `global_costmap/get_costmap`
2. 在机器人周围 `[-pi, pi]` 扫描可退方向
3. 找到一段连续自由空间
4. 取这段自由空间中间角度作为最优后退方向
5. 沿该方向后退 `0.3m`

## 3. 这次做了哪些改动

### 3.1 恢复重试改动

目的：让机器人持续尝试脱困，并更快进入真正的后退恢复，而不是在局部清图里打转太久。

改动如下：

- 外层 `NavigateRecovery.number_of_retries`：`10 -> 2147483647`
- 内层 `FollowPath.number_of_retries`：`10 -> 1`

涉及文件：

- [navigate_to_pose_w_replanning_and_recovery.xml](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml:5)
- [navigate_through_poses_w_replanning_and_recovery.xml](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml:5)

修改后更接近下面的节奏：

1. 跟踪失败
2. 清一次局部图
3. 仍失败就进入外层恢复
4. 在“清图 / 自由空间后退”之间循环
5. 一直重试

### 3.2 脱困可视化改动

目的：让 RViz 里能直接看见脱困插件到底选了哪一片自由空间、准备往哪边退。

可视化实现改在：

- [back_up_free_space.hpp](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb_nav2_plugins/include/pb_nav2_plugins/behaviors/back_up_free_space.hpp:86)
- [back_up_free_space.cpp](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb_nav2_plugins/src/behaviors/back_up_free_space.cpp:257)

新增了这些 marker：

1. `free_points`
   绿色点云，表示搜索半径内的自由点。
2. `robot`
   橙色球，表示当前机器人位置。
3. `direction`
   半透明绿色扇区，表示选中的可退角区间。
4. `planned_backup`
   红色线段，表示当前准备后退的主方向。
5. `direction_text`
   文本标签，显示 `angle / radius / free_pts`。

### 3.3 RViz 配置改动

为了不用每次手动加显示项，已经把新的 marker topic 直接放进默认 RViz 配置：

- [nav2_default_view.rviz](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/rviz/nav2_default_view.rviz:459)

新增显示项：

- 名字：`BackUp Free Space`
- 类型：`MarkerArray`
- topic：`back_up_free_space_markers`

## 4. 如果以后还要继续改，改哪里

### 4.1 想改脱困触发时机

改行为树：

- `navigate_to_pose_w_replanning_and_recovery.xml`
- `navigate_through_poses_w_replanning_and_recovery.xml`

例如：

- 增加 `IsStuck`
- 调整 `RecoveryNode.number_of_retries`
- 调整 `RoundRobin` 里的恢复动作顺序
- 增加 `Spin`、`Wait`、`ClearCostmap` 等恢复节点

### 4.2 想改后退距离 / 速度

改 XML 里的 `BackUp` 参数：

- `backup_dist`
- `backup_speed`

位置：

- [navigate_to_pose_w_replanning_and_recovery.xml](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml:35)
- [navigate_through_poses_w_replanning_and_recovery.xml](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml:40)

### 4.3 想改“自由空间怎么找”

改插件源码：

- [back_up_free_space.cpp](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb_nav2_plugins/src/behaviors/back_up_free_space.cpp:160)

这里可以继续改的点包括：

- 搜索半径 `max_radius`
- 角度分辨率 `M_PI / 32.0`
- 障碍阈值 `costmap.data >= 253`
- 当前使用 `global_costmap/get_costmap`，可改成 `local_costmap/get_costmap`
- 可改成 global/local 联合判断

### 4.4 想改 RViz 显示效果

改插件里的 marker 颜色、大小、样式：

- [back_up_free_space.cpp](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb_nav2_plugins/src/behaviors/back_up_free_space.cpp:257)

或者改默认 RViz 配置：

- [nav2_default_view.rviz](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/rviz/nav2_default_view.rviz:459)

## 5. 为什么之前改了却没生效

之前没生效的核心原因不是配置没改，而是运行中的容器一直在用旧库。

### 5.1 容器用的是挂载工作区

容器创建脚本把宿主机工作区挂到了容器里的 `/root/ros_ws`：

- [recreate_ros_nav_container.sh](/home/asus/nav3_mapping_nogicp_shortLA_stable/recreate_ros_nav_container.sh:48)

也就是说，运行时真正加载的是容器内 `/root/ros_ws/install` 和 `/root/ros_ws/build` 里的结果。

### 5.2 宿主机 build/install 属主是 root

之前 `ros_ws/build` 和 `ros_ws/install` 是 `root:root`，所以在宿主机直接 `colcon build` 会因为权限问题失败，改完源码也不会自动变成新库。

### 5.3 已经在容器里重编过 `pb_nav2_plugins`

本次为了让新可视化真正生效，已经在运行中的容器内执行过：

```bash
docker exec ros_nav_stable_final bash -lc 'cd /root/ros_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select pb_nav2_plugins --symlink-install'
```

## 6. 正确的生效步骤

以后这类改动建议按下面顺序操作。

### 6.1 只改 XML 或 RViz 配置

通常重启导航和 RViz 即可：

1. 停掉 `start_nav_host.sh` 打开的导航相关窗口
2. 重新执行 `./start_nav_host.sh`

### 6.2 改了 `pb_nav2_plugins` 这种 C++ 插件

需要重编插件，再重启导航：

```bash
docker exec ros_nav_stable_final bash -lc 'cd /root/ros_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select pb_nav2_plugins --symlink-install'
```

然后：

1. 关闭 `pb2025_nav_bringup` 窗口
2. 重新执行 `./start_nav_host.sh`
3. 重新发导航目标

## 7. 现在看不到可视化时怎么排查

按这个顺序查最有效。

1. 确认 RViz 里已经有 `BackUp Free Space` 显示项。
2. 确认插件已经在容器里重编，不是旧库。
3. 确认导航进程已经重启，不是老进程。
4. 确认机器人真的进入了 `BackUpFreeSpace` 恢复动作。

最后这一点很重要：

`back_up_free_space_markers` 只有在脱困插件真正执行时才会发布。  
如果导航一直没有进入恢复子树，RViz 里不会有 marker，这是正常现象。

## 8. 后续建议

如果还想继续提升脱困效果，优先建议按这个顺序做：

1. 给恢复树加 `IsStuck`，别等 `FollowPath` 明确失败才恢复。
2. 把 `backup_dist` 从 `0.3` 提到 `0.5` 或 `0.8` 重新测试。
3. 把 `BackUpFreeSpace` 从 `global_costmap` 改成更贴近近场障碍的 `local_costmap`，或者做融合判断。
