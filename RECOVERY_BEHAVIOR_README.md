# 导航恢复行为说明

本文档说明当前实车 Nav2 配置中的恢复行为，重点说明已经启用的 `BackUpFreeSpace` 自由空间后退恢复。这个恢复行为主要用于缓解定位误差导致机器人位姿靠近墙体、进入障碍区或局部无法规划的问题。

## 当前配置

实车 Nav2 参数文件位置：

```text
ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml
```

当前 `backup` 行为配置如下：

```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]

    backup:
      plugin: "pb_nav2_behaviors/BackUpFreeSpace"

    service_name: "global_costmap/get_costmap"
    max_radius: 2.0
    visualize: true
```

对应的导航行为树文件：

```text
ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
```

恢复相关子树如下：

```xml
<ReactiveFallback name="RecoveryFallback">
  <GoalUpdated/>
  <RoundRobin name="RecoveryActions">
    <Sequence name="ClearingActions">
      <ClearEntireCostmap name="ClearLocalCostmap-Subtree"
                          service_name="local_costmap/clear_entirely_local_costmap"/>
      <ClearEntireCostmap name="ClearGlobalCostmap-Subtree"
                          service_name="global_costmap/clear_entirely_global_costmap"/>
    </Sequence>
    <BackUp backup_dist="0.3"
            backup_speed="0.5"/>
  </RoundRobin>
</ReactiveFallback>
```

注意：行为树 XML 里仍然写的是标准 `<BackUp>` 节点，但 `behavior_server` 中 `backup` 这个行为插件已经被配置为 `pb_nav2_behaviors/BackUpFreeSpace`。因此运行到 `<BackUp>` 时，实际执行的是自定义自由空间后退逻辑，而不是 Nav2 默认的固定方向后退。

## 恢复流程

1. 正常导航时，Nav2 在 `NavigateWithReplanning` 中执行全局路径规划和局部路径跟踪。

2. 全局规划失败时，`ComputePathToPose` 对应的恢复节点会清除全局代价地图：

```text
global_costmap/clear_entirely_global_costmap
```

3. 局部路径跟踪失败时，`FollowPath` 对应的恢复节点会清除局部代价地图：

```text
local_costmap/clear_entirely_local_costmap
```

4. 如果主导航流程仍然失败，外层 `NavigateRecovery` 会进入 `RecoveryFallback`。

5. `RecoveryFallback` 会先清除局部和全局代价地图。

6. 如果清图后仍未恢复，行为树会执行：

```xml
<BackUp backup_dist="0.3" backup_speed="0.5"/>
```

7. 由于当前 `backup.plugin` 是 `pb_nav2_behaviors/BackUpFreeSpace`，机器人不会盲目按固定方向后退，而是会在代价地图中搜索附近可通行的自由空间方向，然后以 `0.5 m/s` 的速度移动 `0.3 m`。

## BackUpFreeSpace 工作原理

实现文件：

```text
ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb_nav2_plugins/src/behaviors/back_up_free_space.cpp
```

恢复动作开始时，插件会执行以下步骤：

1. 调用配置中的代价地图服务：

```text
global_costmap/get_costmap
```

2. 获取机器人当前在 `global_frame` 下的位姿。

3. 以机器人当前位置为圆心，在 `-pi` 到 `pi` 范围内扫描各个方向。

4. 对每个方向，从机器人当前位置向外采样，最大检测半径为：

```yaml
max_radius: 2.0
```

5. 如果某个方向上的采样点遇到致命障碍，则认为该方向不可用。代码中的判断阈值是：

```cpp
costmap.data[i + j * size_x] >= 253
```

6. 在所有方向中寻找连续角度范围最大的安全扇区。

7. 选择该安全扇区的中间方向作为最终后退方向。

8. 按行为树传入的距离和速度执行移动：

```xml
backup_dist="0.3"
backup_speed="0.5"
```

这个逻辑的目的，是在定位漂移导致机器人位姿靠近墙体或障碍区时，让机器人优先朝附近自由空间移动，而不是简单原地清图或固定方向倒车。

## 可视化

当前已经开启可视化：

```yaml
visualize: true
```

插件发布的话题：

```text
/back_up_free_space_markers
```

消息类型：

```text
visualization_msgs/msg/MarkerArray
```

在 RViz 中查看：

1. 添加 `MarkerArray` 显示项。
2. topic 选择 `/back_up_free_space_markers`。
3. Fixed Frame 建议使用 `map`，因为当前恢复行为使用的全局坐标系是 `map`。

可视化内容：

- 半透明绿色扇区：检测到的可通行自由空间角度范围。
- 蓝色箭头：安全扇区边界。
- 绿色箭头：最终选择的后退方向。

这些 marker 只会在恢复行为实际运行时发布。正常导航过程中如果没有触发 recovery，RViz 中可能看不到该话题或看不到 marker 更新。

## 与代价地图的关系

当前自定义代价地图层 `IntensityVoxelLayer` 也启用了机器人 footprint 清除：

```text
ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb_nav2_plugins/src/layers/intensity_voxel_layer.cpp
```

实车配置中局部和全局 costmap 都打开了：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      intensity_voxel_layer:
        footprint_clearing_enabled: true

global_costmap:
  global_costmap:
    ros__parameters:
      intensity_voxel_layer:
        footprint_clearing_enabled: true
```

这个机制可以减少机器人把自身点云误标成障碍物的问题。但如果定位误差导致机器人位姿落到静态地图墙体中，单靠 footprint clearing 不一定能解决，因为静态地图层仍然认为该位置是障碍。`BackUpFreeSpace` 的作用就是在 recovery 阶段尝试朝附近自由空间移动，让系统重新获得可规划状态。

## 使用注意事项

- 修改 `nav2_params.yaml` 后，需要重启 Nav2，至少需要重启 `behavior_server`，参数才会重新加载。
- 恢复行为只有在规划失败、路径跟踪失败或主导航流程进入 recovery 时才会运行。
- 如果 RViz 中没有 marker，先确认 recovery 是否真的被触发。
- 如果启动时报插件加载失败，需要确认 `pb_nav2_plugins` 已经在运行环境中正确编译、安装并 source。
- 如果在 Docker 中运行，需要确认容器里使用的是包含该配置修改的源码或 install 空间。

## 常用检查命令

检查当前配置是否启用了 `BackUpFreeSpace`：

```bash
rg -n "BackUpFreeSpace|backup:" ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml
```

检查可视化话题是否存在：

```bash
ros2 topic list | rg back_up_free_space
```

查看 marker 消息：

```bash
ros2 topic echo /back_up_free_space_markers
```

查看 behavior server 日志中是否出现类似输出：

```text
backing up 0.300000 meters towards free space at angle ...
```
