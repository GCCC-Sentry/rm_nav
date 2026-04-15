"""Launch file for odom_to_px4 node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'odom_topic',
            default_value='odometry',
            description='ROS 2 里程计话题名 (nav_msgs/Odometry)',
        ),
        DeclareLaunchArgument(
            'px4_odom_topic',
            default_value='/fmu/in/vehicle_visual_odometry',
            description='PX4 VehicleOdometry 发布话题',
        ),
        DeclareLaunchArgument(
            'use_timesync',
            default_value='true',
            description='是否使用 PX4 时间同步',
        ),
        DeclareLaunchArgument(
            'convert_enu_to_ned',
            default_value='true',
            description='是否进行 ENU→NED / FLU→FRD 坐标系转换',
        ),

        Node(
            package='odom_to_px4',
            executable='odom_to_px4_node',
            name='odom_to_px4_node',
            output='screen',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                'px4_odom_topic': LaunchConfiguration('px4_odom_topic'),
                'use_timesync': LaunchConfiguration('use_timesync'),
                'convert_enu_to_ned': LaunchConfiguration('convert_enu_to_ned'),
            }],
        ),
    ])
