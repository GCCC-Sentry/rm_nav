#!/usr/bin/env python3
"""Launch ADAM 2026UC 哨兵决策节点"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adam_2026uc_decision_v2',
            executable='alliance_decision_node',
            name='alliance_decision_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'hp_supply_threshold': 201,    # HP 低于或等于此值前往补给
                'ammo_supply_threshold': 20,   # 17mm 允许发弹量低于此值前往补给
                'hp_full': 400,                # 满血值
                'supply_timeout_sec': 5.0,     # 补给点最多待 5 秒
                'patrol_interval_sec': 5.0,    # 巡逻点切换间隔
                'spin_speed': 7.0,             # 小陀螺转速 rad/s
                'loop_rate_hz': 10.0,          # 主循环频率
                'defend_stance_duration_sec': 60.0,   # 防御 60 秒后切到移动
                'move_stance_duration_sec': 60.0,     # 移动 60 秒后恢复防御
            }],
        ),
    ])
