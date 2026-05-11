from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    tf_nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.075", "0.0", "-0.071", "0.0", "-0.13062563953108347", "0.0", "0.9914317638128686", "armor_support_frame_0", "armor_0"],
            
            name="armor_support_frame_0_to_armor_0",
            
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.075", "0.0", "-0.071", "0.0", "-0.13062563953108347", "0.0", "0.9914317638128686", "armor_support_frame_1", "armor_1"],
            
            name="armor_support_frame_1_to_armor_1",
            
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.075", "0.0", "-0.071", "0.0", "-0.13062563953108347", "0.0", "0.9914317638128686", "armor_support_frame_2", "armor_2"],
            
            name="armor_support_frame_2_to_armor_2",
            
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.075", "0.0", "-0.071", "0.0", "-0.13062563953108347", "0.0", "0.9914317638128686", "armor_support_frame_3", "armor_3"],
            
            name="armor_support_frame_3_to_armor_3",
            
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.197", "0.0", "0.061", "0.0", "0.0", "0.0", "1.0", "chassis", "armor_support_frame_0"],
            
            name="chassis_to_armor_support_frame_0",
            
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.0", "0.116", "0.061", "0.0", "0.0", "0.7071080798594737", "0.7071054825112364", "chassis", "armor_support_frame_1"],
            
            name="chassis_to_armor_support_frame_1",
            
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["-0.197", "0.0", "0.061", "0.0", "0.0", "0.9999996829318346", "0.0007963267107332633", "chassis", "armor_support_frame_2"],
            
            name="chassis_to_armor_support_frame_2",
            
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.0", "-0.116", "0.061", "0.0", "0.0", "-0.7071080798594737", "0.7071054825112364", "chassis", "armor_support_frame_3"],
            
            name="chassis_to_armor_support_frame_3",
            
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0", "base_footprint", "base_link"],

            name="base_footprint_to_base_link",

        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.0", "0.0", "0.076", "0.0", "0.0", "0.0", "1.0", "base_footprint", "chassis"],

            name="base_footprint_to_chassis",

        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.1", "0.0", "0.045", "0.0", "0.0", "0.0", "1.0", "gimbal_pitch", "front_industrial_camera"],
            
            name="gimbal_pitch_to_front_industrial_camera",
            
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.0", "0.0", "0.0", "0.5", "-0.4999999999999999", "0.5", "-0.5000000000000001", "front_industrial_camera", "front_industrial_camera_optical_frame"],
            
            name="front_industrial_camera_to_optical_frame",
            
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.16", "0.0", "0.18", "1.0", "0.0", "0.0", "0.0", "chassis", "front_mid360"],
            
            name="chassis_to_front_mid360",
            
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.155", "0.0", "0.1", "1.0", "0.0", "0.0", "0.0", "chassis", "front_rplidar_a2"],
            
            name="chassis_to_front_rplidar_a2",
            
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["-0.207", "0.0", "0.1", "0.0", "0.0", "0.0", "1.0", "chassis", "light_indicator"],
            
            name="chassis_to_light_indicator",
            
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.07", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0", "gimbal_pitch", "speed_monitor"],
            
            name="gimbal_pitch_to_speed_monitor",
            
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.07", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0", "chassis", "gimbal_yaw"],
            
            name="chassis_to_gimbal_yaw",
            
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.07", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0", "gimbal_yaw", "gimbal_yaw_fake"],
            
            name="gimbal_yaw_to_gimbal_yaw_fake",
            
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.0", "0.0", "0.0", "0.01", "0.0", "0.0", "0.0", "gimbal_yaw", "gimbal_pitch"],
            
            name="gimbal_yaw_to_gimbal_pitch",
            
        ),
    ]

    return LaunchDescription(tf_nodes)
