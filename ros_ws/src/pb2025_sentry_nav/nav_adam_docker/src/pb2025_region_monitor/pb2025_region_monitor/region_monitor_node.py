import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Twist
from std_msgs.msg import Float32, Int8
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sentry_interfaces.msg import EnemyTargetArray


# ================= 颠簸区域配置 =================
# 每个区域包含:
#   name      : 区域名称 (日志/可视化)
#   vertices  : 多边形顶点 [(x,y), ...], map坐标系, 按顺序排列
#   target_yaw: 穿越时 yaw 目标角度 (弧度, map坐标系, 0 = +X方向)
#   forward_speed: 穿越时强制前进速度 (m/s, 正值=车头方向)
BUMP_ZONES = [
    {
        'name': 'bump_zone_1',
        'vertices': [
                (-5.77, -3.68),
                (-5.84, -6.06),
                (1.07, -6.00),
                (1.38, -3.96),
        ],
        'target_yaw': 0.0,       # yaw=0 即 map +X 方向
        'forward_speed': 1.0,    # 前进速度 m/s
    },
    # 可继续添加更多颠簸区域:
    # {
    #     'name': 'bump_zone_2',
    #     'vertices': [(5.0, 2.0), (7.0, 2.0), (7.0, 4.0), (5.0, 4.0)],
    #     'target_yaw': 1.57,   # +Y方向穿越
    #     'forward_speed': 1.0,
    # },
]

# 颠簸区域覆盖发送频率 (Hz)
BUMP_OVERRIDE_RATE = 50.0

# 进入颠簸区域时发给 /cmd_chassis_mode 的 running_state 值
# 注意: running_state=5 专用于颠簸区域底盘 yaw 对齐, 与姿态切换 (1/2/3) 是两套独立逻辑
BUMP_RUNNING_STATE = 5  # 颠簸区域底盘对齐模式 (非姿态切换)
# ================================================


def normalize_angle(angle):
    """将角度归一化到 [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(q):
    """从四元数提取 yaw 角 (绕 Z 轴旋转)"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RegionMonitorNode(Node):
    def __init__(self):
        super().__init__('region_monitor_node')

        # ================= 普通监控区域 (保留原有功能) =================
        self.regions = [
            {
                'name': 'patrol_zone_1',
                'vertices': [
                    (-1.77, -0.68),
                    (-1.84, -0.06),
                    (0.07, -0.01),
                    (0.38, -.96),
                ]
            },
            # 您可以添加更多普通监控区域...
        ]
        # ===================================================================

        # 发布可视化 Marker 的话题
        self.marker_pub = self.create_publisher(MarkerArray, '/region_markers', 10)

        # ===== 颠簸区域穿越: 发布控制指令 =====
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.yaw_angle_pub = self.create_publisher(Float32, '/cmd_yaw_angle', 10)
        self.chassis_mode_pub = self.create_publisher(Int8, '/cmd_chassis_mode', 10)
        self.enemy_targets_sub = self.create_subscription(
            EnemyTargetArray, '/enemy_targets', self.enemy_targets_callback, 10)
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        # TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 状态记录
        self.in_region_prev = False        # 普通区域
        self.in_bump_zone = False          # 颠簸区域
        self.active_bump_zone = None       # 当前激活的颠簸区域配置
        self.current_yaw = 0.0
        self.latest_enemy_yaw = None
        self.latest_nav_yaw = None
        self.pose_valid = False

        # 定时器: 10Hz 检查区域 + 发布可视化
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 定时器: 高频颠簸区域覆盖 (默认不启动, 进入区域时动态创建)
        self.bump_override_timer = None

        self.get_logger().info(
            f'区域监控节点已启动: {len(self.regions)} 个普通区域, '
            f'{len(BUMP_ZONES)} 个颠簸区域')

    @staticmethod
    def point_in_polygon(px, py, vertices):
        """射线法判断点是否在多边形内"""
        n = len(vertices)
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = vertices[i]
            xj, yj = vertices[j]
            if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside

    def timer_callback(self):
        try:
            # 1. 获取机器人坐标和朝向
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())

            current_x = t.transform.translation.x
            current_y = t.transform.translation.y
            self.current_yaw = quaternion_to_yaw(t.transform.rotation)
            self.pose_valid = True
            self._publish_selected_yaw()

            # 2. 检查普通监控区域 + 发布可视化
            in_any_region = False
            marker_array = MarkerArray()
            timestamp = self.get_clock().now().to_msg()

            for i, region in enumerate(self.regions):
                verts = region['vertices']
                is_inside_this = self.point_in_polygon(current_x, current_y, verts)
                if is_inside_this:
                    in_any_region = True

                # 构建可视化 Marker (普通区域: 绿/红)
                marker = self._make_polygon_marker(
                    i, "monitor_regions", verts, timestamp,
                    inside=is_inside_this,
                    color_in=(1.0, 0.0, 0.0),    # 红
                    color_out=(0.0, 1.0, 0.0))    # 绿
                marker_array.markers.append(marker)

            # 3. 检查颠簸区域 + 发布可视化
            prev_in_bump = self.in_bump_zone
            self.in_bump_zone = False
            self.active_bump_zone = None

            for i, zone in enumerate(BUMP_ZONES):
                verts = zone['vertices']
                is_inside_this = self.point_in_polygon(current_x, current_y, verts)
                if is_inside_this:
                    self.in_bump_zone = True
                    self.active_bump_zone = zone

                # 颠簸区域可视化 (蓝/橙)
                marker = self._make_polygon_marker(
                    100 + i, "bump_zones", verts, timestamp,
                    inside=is_inside_this,
                    color_in=(1.0, 0.5, 0.0),    # 橙 (激活)
                    color_out=(0.0, 0.5, 1.0),    # 蓝 (未激活)
                    line_width=0.08)
                marker_array.markers.append(marker)

            self.marker_pub.publish(marker_array)

            # 4. 普通区域日志
            if in_any_region and not self.in_region_prev:
                self.get_logger().warn('>>> 进入监控区域')
            elif not in_any_region and self.in_region_prev:
                self.get_logger().info('<<< 离开监控区域')
            self.in_region_prev = in_any_region

            # 5. 颠簸区域进入/离开处理
            if self.in_bump_zone and not prev_in_bump:
                self._on_enter_bump_zone()
            elif not self.in_bump_zone and prev_in_bump:
                self._on_leave_bump_zone()

        except TransformException:
            pass

    # ================= 颠簸区域进入/离开逻辑 =================

    def _on_enter_bump_zone(self):
        """进入颠簸区域: 启动高频覆盖定时器"""
        zone = self.active_bump_zone

        self.get_logger().warn(
            f'>>> 进入颠簸区域 [{zone["name"]}]! '
            f'强制 running_state={BUMP_RUNNING_STATE}, '
            f'x={zone["forward_speed"]}, y=0, '
            f'当前yaw={math.degrees(self.current_yaw):.1f}°')

        # 发布 running_state 切换
        mode_msg = Int8()
        mode_msg.data = BUMP_RUNNING_STATE
        self.chassis_mode_pub.publish(mode_msg)

        # 启动高频覆盖定时器
        if self.bump_override_timer is None:
            self.bump_override_timer = self.create_timer(
                1.0 / BUMP_OVERRIDE_RATE, self._bump_override_callback)

    def _on_leave_bump_zone(self):
        """离开颠簸区域: 停止覆盖, 恢复底盘模式"""
        self.get_logger().info('<<< 离开颠簸区域, 恢复正常控制')

        # 停止覆盖定时器
        if self.bump_override_timer is not None:
            self.bump_override_timer.cancel()
            self.destroy_timer(self.bump_override_timer)
            self.bump_override_timer = None

        # 恢复底盘模式 (发送 0 表示交还给姿态系统)
        mode_msg = Int8()
        mode_msg.data = 0
        self.chassis_mode_pub.publish(mode_msg)

        # 发送一次零速度, 防止惯性
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

    def _bump_override_callback(self):
        """高频定时器: 在颠簸区域内持续发布强制速度指令和目标 yaw 角度。"""
        if not self.in_bump_zone or self.active_bump_zone is None:
            return

        zone = self.active_bump_zone

        # 颠簸区域: 发布前进速度通过 cmd_vel, yaw 由独立角度话题发
        twist = Twist()
        twist.linear.x = zone['forward_speed']
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

        self._publish_selected_yaw()

        self.cmd_vel_pub.publish(twist)

        # 持续刷新 running_state (确保串口节点保持 running_state=5)
        mode_msg = Int8()
        mode_msg.data = BUMP_RUNNING_STATE
        self.chassis_mode_pub.publish(mode_msg)

    def enemy_targets_callback(self, msg):
        if not msg.targets:
            return
        self.latest_enemy_yaw = float(msg.targets[0].yaw)

    def goal_pose_callback(self, msg):
        self.latest_nav_yaw = quaternion_to_yaw(msg.pose.orientation)

    def _publish_selected_yaw(self):
        if self.in_bump_zone:
            selected_yaw = self.latest_nav_yaw
            if selected_yaw is None and self.active_bump_zone is not None:
                selected_yaw = float(self.active_bump_zone['target_yaw'])
        else:
            selected_yaw = self.latest_enemy_yaw
            if selected_yaw is None:
                selected_yaw = self.latest_nav_yaw

        if selected_yaw is None:
            selected_yaw = self.current_yaw

        yaw_msg = Float32()
        yaw_msg.data = float(normalize_angle(selected_yaw))
        self.yaw_angle_pub.publish(yaw_msg)

    # ================= 可视化辅助 =================

    def _make_polygon_marker(self, marker_id, ns, vertices, timestamp,
                             inside=False,
                             color_in=(1.0, 0.0, 0.0),
                             color_out=(0.0, 1.0, 0.0),
                             line_width=0.05):
        """创建多边形线条 Marker"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = timestamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = line_width

        if inside:
            marker.color.r, marker.color.g, marker.color.b = color_in
        else:
            marker.color.r, marker.color.g, marker.color.b = color_out
        marker.color.a = 1.0

        for vx, vy in vertices:
            marker.points.append(Point(x=vx, y=vy, z=0.0))
        marker.points.append(Point(x=vertices[0][0], y=vertices[0][1], z=0.0))

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 200000000  # 0.2s
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = RegionMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
