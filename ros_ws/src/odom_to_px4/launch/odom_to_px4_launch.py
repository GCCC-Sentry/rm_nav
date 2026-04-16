"""Launch file for odom_to_px4 node (MAVLink 串口模式).

数据链路：
  ROS 2 odom_to_px4_node  ──(pymavlink ODOMETRY #331)──>  串口  ──>  PX4 飞控

无需 uXRCE-DDS Agent，节点直接通过串口发送 MAVLink 消息。

常用串口设备:
  /dev/ttyUSB0  - USB-TTL 转换器 → 飞控 TELEM2
  /dev/ttyACM0  - USB 直连 Pixhawk
  /dev/ttyTHS1  - Jetson UART
  /dev/ttyAMA0  - 树莓派 UART
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # =============== 串口参数 ===============
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='连接 PX4 飞控的串口设备路径',
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='921600',
            description='串口波特率 (需与 PX4 TELEM 端口一致)',
        ),

        # =============== ROS 2 节点参数 ===============
        DeclareLaunchArgument(
            'odom_topic',
            default_value='odometry',
            description='ROS 2 里程计话题名 (nav_msgs/Odometry)',
        ),
        DeclareLaunchArgument(
            'convert_enu_to_ned',
            default_value='true',
            description='是否进行 ENU→NED / FLU→FRD 坐标系转换',
        ),
        DeclareLaunchArgument(
            'debug_print_rate',
            default_value='1.0',
            description='调试信息打印频率 (Hz), 0=关闭',
        ),
        DeclareLaunchArgument(
            'mav_sys_id',
            default_value='1',
            description='MAVLink system ID',
        ),
        DeclareLaunchArgument(
            'mav_comp_id',
            default_value='197',
            description='MAVLink component ID (197=VISUAL_INERTIAL_ODOMETRY)',
        ),

        # =============== 打印配置信息 ===============
        LogInfo(msg=['[odom_to_px4] 串口: ', LaunchConfiguration('serial_port'),
                     ' @ ', LaunchConfiguration('baudrate'), ' baud']),
        LogInfo(msg=['[odom_to_px4] 里程计话题: ', LaunchConfiguration('odom_topic')]),
        LogInfo(msg=['[odom_to_px4] 坐标系转换: ', LaunchConfiguration('convert_enu_to_ned')]),

        # =============== odom_to_px4 节点 ===============
        Node(
            package='odom_to_px4',
            executable='odom_to_px4_node',
            name='odom_to_px4_node',
            output='screen',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'convert_enu_to_ned': LaunchConfiguration('convert_enu_to_ned'),
                'debug_print_rate': LaunchConfiguration('debug_print_rate'),
                'mav_sys_id': LaunchConfiguration('mav_sys_id'),
                'mav_comp_id': LaunchConfiguration('mav_comp_id'),
            }],
        ),
    ])
