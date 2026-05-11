# 导航脱困说明

## 当前启动链路

当前主目录脚本 [start_nav_host.sh](/home/asus/nav3_mapping_nogicp_shortLA_stable/start_nav_host.sh:63) 会在容器里启动：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=qian1 slam:=False use_robot_state_pub:=False
```

这条启动链路会默认加载：

- `config/reality/nav2_params.yaml`
- `bt_navigator.default_nav_to_pose_bt_xml = behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`
- `bt_navigator.default_nav_through_poses_bt_xml = behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml`

所以，只要你的上层是往 `navigate_to_pose` 或 `navigate_through_poses` 发目标，现在执行导航时就会走这套恢复树。

## 现在是不是“自动搜寻自由空间”脱困

是，但有前提。

行为树里的恢复动作使用了 `<BackUp backup_dist="0.3" backup_speed="0.5"/>`，而 `behavior_server.backup` 实际绑定到的不是 Nav2 默认后退，而是自定义插件 `pb_nav2_behaviors/BackUpFreeSpace`。

这意味着进入恢复子树后，机器人会：

1. 读取 `global_costmap/get_costmap`
2. 在机器人周围扫描一圈可退方向
3. 选取一段连续自由空间的中间角度
4. 朝这个方向退 `0.3 m`

对应文件：

- [nav2_params.yaml](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml:609)
- [navigate_to_pose_w_replanning_and_recovery.xml](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml:1)
- [back_up_free_space.cpp](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb_nav2_plugins/src/behaviors/back_up_free_space.cpp:54)

## 为什么你测着像没效果

主要有 4 个原因。

1. 恢复动作不是“卡住立刻触发”。
   这棵树里没有用 `IsStuck` 条件节点。它只有在 `FollowPath` 或 `ComputePath` 明确返回失败后，才会进入恢复子树。

2. 之前 `FollowPath` 内层重试太多。
   原来 `FollowPath` 的 `RecoveryNode` 是 `number_of_retries="10"`，它会优先反复执行“清局部代价图 -> 再次跟踪”，未必很快进入真正的 `BackUpFreeSpace`。

3. 真正的后退距离太短。
   当前 `BackUp` 只退 `0.3 m`。如果车体被卡得更深，或者底盘/障碍物接触比较紧，这个量可能看起来几乎没变化。

4. 自由空间判断依赖 `global_costmap`。
   `BackUpFreeSpace` 当前读的是 `global_costmap/get_costmap`。如果你遇到的是近场临时障碍、局部点云障碍、或者全局图没有及时反映当前卡住状态，选出来的“自由方向”就可能不理想。

## 这次已做的修改

为了让它持续尝试脱困，并更快进入真正的后退恢复，我已经修改了两棵导航树：

- 外层 `NavigateRecovery.number_of_retries`：`10 -> 2147483647`
- `FollowPath.number_of_retries`：`10 -> 1`

修改文件：

- [navigate_to_pose_w_replanning_and_recovery.xml](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml:5)
- [navigate_through_poses_w_replanning_and_recovery.xml](/home/asus/nav3_mapping_nogicp_shortLA_stable/ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml:5)

修改后的行为更接近：

1. 跟踪失败
2. 清一次局部代价图
3. 不行就立刻进入外层恢复
4. 在“清图”和“自由空间后退”之间循环
5. 一直重试，不主动放弃

## 生效方式

如果当前工作区是 `colcon build --symlink-install`，通常重启导航进程即可生效；不需要重新编译 XML。

建议直接重启 [start_nav_host.sh](/home/asus/nav3_mapping_nogicp_shortLA_stable/start_nav_host.sh:1) 启动的导航窗口，重新发一次导航目标测试。

## 如果还想继续增强

如果你下一步要把“看起来没效果”的问题继续压下去，建议优先改这三项：

1. 给恢复树加 `IsStuck` 条件，别等 `FollowPath` 明确失败才触发恢复。
2. 把 `BackUp` 的距离从 `0.3` 提到 `0.5` 或 `0.8` 再测。
3. 把 `BackUpFreeSpace` 的取图依据从纯 `global_costmap` 改成更贴近近场障碍的局部代价图，或者做 global/local 融合。
