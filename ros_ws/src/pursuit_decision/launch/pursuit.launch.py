import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('pursuit_decision')
    params_file = os.path.join(pkg_dir, 'config', 'pursuit_params.yaml')

    # 不在 launch 中强制设置 RMW_IMPLEMENTATION。
    # 否则当运行环境未安装 rmw_cyclonedds_cpp 时，节点会在启动阶段直接退出。
    # 如果后续跨版本 DDS 联调确实需要 CycloneDDS，请在启动前显式导出：
    #   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    # 前提是对应运行环境已经安装了 librmw_cyclonedds_cpp.so。
    pursuit_node = Node(
        package='pursuit_decision',
        executable='pursuit_node',
        name='pursuit_decision',
        output='screen',
        parameters=[params_file],
        remappings=[
            # 自瞄发来的目标信息
            ('auto_aim_target_pos', 'auto_aim_target_pos'),
            # 裁判系统状态
            ('pursuit/robot_status', 'referee/robot_status'),
            ('pursuit/game_status', 'referee/game_status'),
            # 导航代价地图
            ('global_costmap/costmap', 'global_costmap/costmap'),
            # 发出的导航目标
            ('goal_pose', 'goal_pose'),
            # 单独可视化的目标点和攻击点
            ('pursuit_target_pose', 'pursuit_target_pose'),
            ('pursuit_attack_pose', 'pursuit_attack_pose'),
            ('pursuit_nav_goal_pose', 'pursuit_nav_goal_pose'),
            # 追击状态
            ('pursuit_status', 'pursuit_status'),
            # RViz 调试可视化
            ('pursuit_debug_markers', 'pursuit_debug_markers'),
            ('pursuit_goal_history', 'pursuit_goal_history'),
        ],
    )

    return LaunchDescription([pursuit_node])
