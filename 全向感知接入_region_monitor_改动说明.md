# 全向感知接入 `region_monitor` 改动说明

## 1. 改动目标

本次改动将 `sentry_omni26` 的全向感知输出接入
`nav3_mapping_nogicp_shortLA_stable` 的导航链路中，使：

- 非颠簸路段区域时，`region_monitor_node` 使用全向感知的目标方位角逻辑。
- 进入颠簸路段区域时，继续保持原先的底盘对齐逻辑，不使用全向感知控制大 yaw。
- 下位机发送链路继续沿用现有方案：
  `region_monitor_node -> /cmd_yaw_angle -> my_serial_py -> 下位机`

## 2. 全向感知坐标系说明

`sentry_omni26` 当前融合后的目标话题为：

- 话题名：`/enemy_targets`
- 消息类型：`sentry_interfaces/msg/EnemyTargetArray`
- 坐标系：`header.frame_id = "base_link"`

代码中使用的是 ROS 常见车体系定义：

- `x` 正方向：车头前方
- `y` 正方向：车体左侧
- `z` 正方向：车体上方

单个目标中的 `yaw` 由下面公式得到：

```cpp
yaw = atan2(y, x)
```

因此它表示：

- 目标相对于当前车头正前方的方位角
- 车头正前 = `0`
- 目标在左侧时 `yaw > 0`
- 目标在右侧时 `yaw < 0`

这与本车当前约定一致：

- 车头正前 = `0`

## 3. 改动内容

### 3.1 引入全向感知消息包

为了让导航工作区能够直接订阅 `/enemy_targets`，
已将 `sentry_omni26/sentry_interfaces` 复制到导航工作区：

- 目标目录：`ros_ws/src/sentry_interfaces`

这样导航容器编译后即可识别：

- `sentry_interfaces/msg/EnemyTarget`
- `sentry_interfaces/msg/EnemyTargetArray`

### 3.2 `region_monitor_node` 增加全向感知订阅

新增订阅：

- `/enemy_targets`
- 类型：`sentry_interfaces/msg/EnemyTargetArray`

同时保留旧的：

- `/tracker/target`
- 类型：`auto_aim_interfaces/msg/Target`

这样做的目的：

- 主逻辑切到全向感知
- 老接口先不删，便于过渡和回退

### 3.3 非颠簸区使用全向感知逻辑

当车辆 **不在颠簸区域** 时：

- 优先使用 `/enemy_targets` 中的目标 `yaw`
- 若全向感知暂时无数据，再回退旧 `/tracker/target`
- 若仍无数据，再回退 `goal_pose`

目标选择规则为：

1. 优先 `is_tracking == true`
2. 再按 `priority` 更高优先
3. 再按 `confidence` 更高优先

注意：

- 全向感知的 `yaw` 已经是相对 `base_link` 的角度
- 因此这里**直接转成角度后发送**
- 不再走原先“减去启动基准角”的绝对角换算

### 3.4 颠簸区保持原逻辑

当车辆 **进入颠簸区域** 时：

- 继续使用原先底盘对齐逻辑
- `running_state = 5`
- `cmd_yaw_angle` 继续按原有“回到启动基准角”的逻辑计算
- 不使用全向感知 `yaw`

因此本次改动满足：

- 非颠簸区：全向感知逻辑
- 颠簸区：原先底盘对齐逻辑

### 3.5 启动脚本补充 `region_monitor_node`

已修改：

- `start_nav_host.sh`

新增启动窗口：

- `pb2025_region_monitor`

启动命令：

```bash
ros2 run pb2025_region_monitor region_monitor_node
```

这样执行 `start_nav_host.sh` 时，会自动把 `region_monitor_node` 一起拉起。

## 4. 修改文件清单

- `start_nav_host.sh`
- `ros_ws/src/sentry_interfaces/*`
- `ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_region_monitor/package.xml`
- `ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_region_monitor/pb2025_region_monitor/region_monitor_node.py`

## 5. 使用注意

### 5.1 需要重新编译导航工作区

因为新增了 `sentry_interfaces` 消息包，导航容器内必须重新编译：

```bash
cd /root/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

如果暂时还没有编译 `sentry_interfaces`：

- `region_monitor_node` 现在也可以启动
- 节点会自动回退到旧 `/tracker/target` 逻辑
- 只有在 `sentry_interfaces` 编译并 `source install/setup.bash` 之后，才会启用 `/enemy_targets` 全向感知订阅

### 5.2 全向感知与导航需在同一 ROS2 网络

需要保证：

- `/enemy_targets` 确实能被导航侧看到
- 两边 ROS_DOMAIN_ID 一致
- 网络互通正常

### 5.3 当前下位机链路仍沿用旧字段语义

虽然口头上常说“发 yawspeed”，
但当前实际代码仍然是：

- `region_monitor_node` 发布 `/cmd_yaw_angle`
- `my_serial_py` 将该角度写入原报文中的 yaw 相关槽位

也就是说：

- 本次没有改串口协议结构
- 只是把非颠簸区的大 yaw 目标来源切换为全向感知

## 6. 建议验证步骤

1. 重新编译导航容器工作区。
2. 启动导航和全向感知。
3. `ros2 topic echo /enemy_targets`，确认有目标数据。
4. `ros2 topic echo /cmd_yaw_angle`，确认非颠簸区时角度随全向感知目标变化。
5. 开进颠簸区域，确认 `/cmd_chassis_mode` 切到 `5`。
6. 颠簸区域内确认 `/cmd_yaw_angle` 回到原先对齐逻辑，而不是跟随全向感知。
7. 离开颠簸区域后，确认恢复全向感知逻辑。

## 7. 当前实现结论

本次接入后的控制策略是：

- 非颠簸区：大 yaw 跟随全向感知目标方位角
- 颠簸区：大 yaw 保持原先底盘对齐策略
- 下位机接口与串口发送结构不变
