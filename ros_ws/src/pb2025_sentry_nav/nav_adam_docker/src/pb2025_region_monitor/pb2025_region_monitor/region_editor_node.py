import json
import os
from copy import deepcopy

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from pb2025_region_monitor.region_monitor_node import DEFAULT_BUMP_ZONES, DEFAULT_ENERGY_ZONES


class RegionEditorNode(Node):
    def __init__(self):
        super().__init__('region_editor_node')

        default_output_path = os.path.expanduser('~/region_monitor_regions.json')
        self.declare_parameter('output_path', default_output_path)
        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value

        config_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/region_editor_markers', 10)
        self.config_pub = self.create_publisher(String, '/region_editor/config', config_qos)
        self.clicked_point_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        self.command_sub = self.create_subscription(
            String, '/region_editor/command', self.command_callback, 10)

        self.current_mode = 'monitor_forward'
        self.current_name = 'region_1'
        self.pending_points = []
        self.monitor_regions = [
            {
                'name': 'direction_zone_forward',
                'vertices': [
                    (-3.19, 1.24),
                    (-4.94, 1.36),
                    (-4.87, 0.16),
                    (-3.14, 0.36),
                ],
                'direction_code': 1,
            },
            {
                'name': 'direction_zone_backward',
                'vertices': [
                    (-6.90, 1.28),
                    (-7.53, 1.70),
                    (-7.80, 0.40),
                    (-5.50, 1.86),
                ],
                'direction_code': 2,
            },
            {
                'name': 'direction_zone_backward_2',
                'vertices': [
                    (-5.55, 1.55),
                    (-7.10, 1.35),
                    (-6.95, 0.35),
                    (-5.20, 0.45),
                ],
                'direction_code': 2,
            },
        ]
        self.bump_zones = deepcopy(DEFAULT_BUMP_ZONES)
        self.energy_zones = deepcopy(DEFAULT_ENERGY_ZONES)
        self.region_counter = 1

        self.timer = self.create_timer(0.1, self.publish_markers)

        self.get_logger().info(
            '区域编辑器已启动。'
            '在 RViz 使用 Publish Point 工具点击四个点即可生成一个区域。'
            '命令话题: /region_editor/command')
        self.get_logger().info(
            '可用命令: set monitor_forward | set monitor_backward | set bump | set energy | '
            'name 区域名 | undo | clear_points | clear_regions | save | show')
        self._publish_config()

    def clicked_point_callback(self, msg):
        point = (round(float(msg.point.x), 3), round(float(msg.point.y), 3))
        self.pending_points.append(point)
        self.get_logger().info(
            f'已记录点 {len(self.pending_points)}/4: ({point[0]:.3f}, {point[1]:.3f})')

        if len(self.pending_points) == 4:
            self._commit_region()

    def command_callback(self, msg):
        command = msg.data.strip()
        if not command:
            return

        if command.startswith('set '):
            mode = command[4:].strip()
            if mode not in ('monitor_forward', 'monitor_backward', 'bump', 'energy'):
                self.get_logger().warn(
                    '未知模式。可选: monitor_forward, monitor_backward, bump, energy')
                return
            self.current_mode = mode
            self.get_logger().info(f'当前绘制模式已切换为: {self.current_mode}')
            return

        if command.startswith('name '):
            name = command[5:].strip()
            if not name:
                self.get_logger().warn('区域名不能为空')
                return
            self.current_name = name
            self.get_logger().info(f'当前区域名已切换为: {self.current_name}')
            return

        if command == 'undo':
            if self.pending_points:
                removed = self.pending_points.pop()
                self.get_logger().info(
                    f'已撤销最后一个点: ({removed[0]:.3f}, {removed[1]:.3f})')
            else:
                self.get_logger().info('当前没有待提交的点可撤销')
            return

        if command == 'clear_points':
            self.pending_points.clear()
            self.get_logger().info('已清空当前待提交点')
            return

        if command == 'clear_regions':
            self.pending_points.clear()
            self.monitor_regions.clear()
            self.bump_zones.clear()
            self.energy_zones.clear()
            self.region_counter = 1
            self.get_logger().info('已清空全部已绘制区域')
            self._publish_config()
            return

        if command == 'save':
            self._save_to_file()
            return

        if command == 'show':
            self._log_region_summary()
            return

        self.get_logger().warn('未知命令')

    def _commit_region(self):
        if len(self.pending_points) != 4:
            return

        region_name = self.current_name
        if not region_name:
            region_name = f'region_{self.region_counter}'

        vertices = deepcopy(self.pending_points)
        self.pending_points.clear()
        self.region_counter += 1

        if self.current_mode == 'bump':
            region = {
                'name': region_name,
                'vertices': vertices,
            }
            self.bump_zones.append(region)
            self.get_logger().warn(
                f'已创建颠簸区域 {region_name}: {vertices}')
        elif self.current_mode == 'energy':
            region = {
                'name': region_name,
                'vertices': vertices,
            }
            self.energy_zones.append(region)
            self.get_logger().warn(
                f'已创建打符区域 {region_name}: {vertices}')
        else:
            direction_code = 1 if self.current_mode == 'monitor_forward' else 2
            region = {
                'name': region_name,
                'vertices': vertices,
                'direction_code': direction_code,
            }
            self.monitor_regions.append(region)
            self.get_logger().warn(
                f'已创建监控区域 {region_name}，方向='
                f'{"去程" if direction_code == 1 else "回程"}: {vertices}')

        self.current_name = f'region_{self.region_counter}'
        self._log_region_summary()
        self._publish_config()

    def _save_to_file(self):
        payload = {
            'monitor_regions': self.monitor_regions,
            'bump_zones': self.bump_zones,
            'energy_zones': self.energy_zones,
        }
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        with open(self.output_path, 'w', encoding='utf-8') as file_obj:
            json.dump(payload, file_obj, ensure_ascii=False, indent=2)
        self.get_logger().warn(f'区域配置已保存到: {self.output_path}')
        self._publish_config()

    def _log_region_summary(self):
        self.get_logger().info(
            f'当前共 {len(self.monitor_regions)} 个监控区域, '
            f'{len(self.bump_zones)} 个颠簸区域, '
            f'{len(self.energy_zones)} 个打符区域, '
            f'待提交点 {len(self.pending_points)} 个')

    def _publish_config(self):
        payload = {
            'monitor_regions': self.monitor_regions,
            'bump_zones': self.bump_zones,
            'energy_zones': self.energy_zones,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.config_pub.publish(msg)

    def publish_markers(self):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for index, region in enumerate(self.monitor_regions):
            direction_code = int(region.get('direction_code', 0))
            if direction_code == 1:
                color = (0.0, 1.0, 0.0)
            else:
                color = (1.0, 0.0, 1.0)
            marker_array.markers.append(
                self._make_polygon_marker(
                    marker_id=index,
                    ns='editor_monitor_regions',
                    vertices=region['vertices'],
                    timestamp=stamp,
                    color=color,
                    line_width=0.07,
                )
            )

        for index, zone in enumerate(self.bump_zones):
            marker_array.markers.append(
                self._make_polygon_marker(
                    marker_id=1000 + index,
                    ns='editor_bump_zones',
                    vertices=zone['vertices'],
                    timestamp=stamp,
                    color=(0.0, 0.5, 1.0),
                    line_width=0.09,
                )
            )

        for index, zone in enumerate(self.energy_zones):
            marker_array.markers.append(
                self._make_polygon_marker(
                    marker_id=1500 + index,
                    ns='editor_energy_zones',
                    vertices=zone['vertices'],
                    timestamp=stamp,
                    color=(1.0, 0.8, 0.0),
                    line_width=0.09,
                )
            )

        if self.pending_points:
            marker_array.markers.append(
                self._make_points_marker(
                    marker_id=2000,
                    ns='editor_pending_points',
                    points=self.pending_points,
                    timestamp=stamp,
                    color=(1.0, 1.0, 1.0),
                )
            )

        self.marker_pub.publish(marker_array)

    def _make_polygon_marker(self, marker_id, ns, vertices, timestamp, color, line_width):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = timestamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = line_width
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0

        for vx, vy in vertices:
            marker.points.append(Point(x=float(vx), y=float(vy), z=0.0))
        marker.points.append(Point(x=float(vertices[0][0]), y=float(vertices[0][1]), z=0.0))
        return marker

    def _make_points_marker(self, marker_id, ns, points, timestamp, color):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = timestamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0

        for vx, vy in points:
            marker.points.append(Point(x=float(vx), y=float(vy), z=0.0))
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = RegionEditorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
