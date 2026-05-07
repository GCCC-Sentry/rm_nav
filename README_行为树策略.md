# 哨兵行为树策略 README

本文档说明当前仓库中的哨兵行为树体系、当前启用的策略、与 Nav2 的关系，以及后续扩展方式。

## 1. 当前结论

当前仓库里有两层“树”：

1. Nav2 导航行为树  
负责 `navigate_to_pose` / `goal_pose` 的底层导航、重规划、恢复。

2. 哨兵自定义策略行为树  
负责比赛开始判断、路点决策、低血低弹回补给、死亡恢复等上层逻辑。

当前实际启用的上层策略树是：

- `rmul_2026_bump_supply_center`

其配置位置：

- `ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_sentry_behavior/params/sentry_behavior.yaml`

其 XML 文件位置：

- `ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_sentry_behavior/behavior_trees/rmul/rmul_2026_bump_supply_center.xml`

## 2. 当前策略主流程

当前策略流程如下：

1. 等待比赛开始。
2. 开始自旋。
3. 正常状态下，先导航到颠簸区前的集结点。
4. 再发送中心战场目标点。
5. 机器人进入颠簸区域后，由 `pb2025_region_monitor` 自动接管底盘对齐与通过。
6. 穿越完成后，继续前往中心战场。
7. 如果血量低于 `201`，或者 17mm 允许发弹量低于 `20`，则回补给区。
8. 如果死亡，则先等待复活，再回补给区解除“虚弱”并恢复发射机构可用状态。

## 3. 为什么要区分 Nav2 树和策略树

这是当前系统里最容易混淆的点。

### 3.1 Nav2 树负责什么

Nav2 树负责：

- 路径规划
- 路径跟踪
- 恢复行为
- 重规划
- 导航失败处理

也就是说，`SendNav2Goal` 或 `PubNav2Goal` 发出目标后，具体怎么走，是 Nav2 的默认树在执行。

### 3.2 策略树负责什么

策略树负责：

- 什么时候出发
- 去哪个点
- 什么时候回补给
- 死亡后怎么恢复
- 是否进入中心战场
- 是否等待在补给区

所以：

- 改“比赛策略”，优先改哨兵策略树 XML
- 改“导航恢复机制”，才去改 Nav2 的 XML

## 4. 当前策略坐标

当前新策略树内使用的默认坐标为：

- 集结点：`(-2.50, -4.90, 0.00)`
- 穿越后中心战场目标：`(3.80, 0.00, 0.00)`
- 补给点：`(2.80, 0.13, 0.00)`

这些坐标是按当前仓库内既有 RMUL 小坐标系推定写入的，后续实场使用时应优先复核。

## 5. 规则对应关系

根据仓库根目录中的规则手册：

- 存活机器人占领己方补给区时可以回血。
- 哨兵可通过占领己方补给区按分钟获得允许发弹量。
- 读条复活后只恢复部分血量，并进入“虚弱”状态。
- “虚弱”状态下发射机构锁定。
- 重新检测到可占领的补给区/基地增益点/前哨站增益点后，“虚弱”解除。

因此当前策略里“死亡后复活再回补给区”不是随便设计的，而是为了满足规则上的解锁路径。

## 6. 当前已知限制

### 6.1 没有自动裁判系统指令插件

当前仓库中没有看到已接好的行为树节点去执行：

- 确认复活
- 立即复活
- 远程兑换血量
- 远程兑换允许发弹量

所以当前策略是：

- 被动等待复活
- 复活后回补给区解除虚弱
- 在补给区等待回血和弹药恢复

### 6.2 颠簸区域接管依赖 region monitor

策略树本身不直接做底盘对齐控制。  
它的设计是假设：

- `pb2025_region_monitor` 正常运行
- 机器人进入配置好的颠簸区域多边形后
- `region_monitor` 会接管 `cmd_vel`、`cmd_chassis_mode`、`cmd_yaw_angle`

如果这个节点没启动，或者区域配置不对，策略树只能负责发目标点，无法保证通过颠簸路段。

## 7. 推荐改法

如果后续要继续开发，建议按以下优先级改：

1. 先确认坐标点。
2. 再确认补给等待阈值是否符合战术。
3. 再决定是否增加“自动确认复活 / 自动兑换弹丸”的 BT 插件。
4. 最后再考虑加姿态切换、攻击逻辑、视觉驱动逻辑。

## 8. 相关文件

- 当前策略参数  
  `ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_sentry_behavior/params/sentry_behavior.yaml`

- 当前策略树  
  `ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_sentry_behavior/behavior_trees/rmul/rmul_2026_bump_supply_center.xml`

- region monitor 备份实现  
  `ros_ws/src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_region_monitor__orig_backup_20260505/pb2025_region_monitor/region_monitor_node.py`

- 比赛规则手册  
  `RoboMaster 2026 机甲大师超级对抗赛比赛规则手册V1.4.2（20260430）.pdf`
