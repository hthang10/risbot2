#!/usr/bin/env python3

import os
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    rb2_param_dir = LaunchConfiguration(
        'rb2_param_dir',
        default=os.path.join(
            get_package_share_directory('risbot2_bringup'),
            'param',
            'risbot2.yaml'))

    hokuyo_launch_file = LaunchConfiguration(
        'hokuyo_launch_file',
        default=os.path.join(get_package_share_directory('urg_node2'), 'launch', 'urg_node2.launch.py'))

    camera_model = LaunchConfiguration('camera_model', default='zed2')
    publish_tf = LaunchConfiguration('publish_tf', default='false')
    param_overrides = LaunchConfiguration(
        'param_overrides',
        default='body_tracking.bt_enabled:=true;object_detection.od_enabled:=true;object_detection.enable_tracking:=true'
    )

    # ZED Camera Launch
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')
        ]),
        launch_arguments={
            'camera_model': camera_model,
            'publish_tf': publish_tf,
            'publish_urdf': 'true',
            'node_name': 'zed_node',
            'param_overrides': param_overrides,
        }.items(),
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'rb2_param_dir',
            default_value=rb2_param_dir,
            description='Full path to risbot2 parameter file to load'),

        DeclareLaunchArgument(
            'camera_model',
            default_value=camera_model,
            description='Model of the ZED camera (e.g. zed2, zed2i)'),

        DeclareLaunchArgument(
            'publish_tf',
            default_value=publish_tf,
            description='Enable publication of the odom -> camera_link TF by the ZED node'),

        DeclareLaunchArgument(
            'param_overrides',
            default_value=param_overrides,
            description='Parameter overrides for the ZED node'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/risbot2_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([hokuyo_launch_file]),
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_base_link_to_zed',
            output='screen',
            # arguments=['0.0', '0.0', '0.2', '3.14159265', '0.0', '0.0', 'base_link', 'zed_camera_link']
            arguments=['0.0', '0.0', '0.2', '0.0', '0.0', '0.0', 'base_link', 'zed_camera_link']
        ),

        zed_launch,

        Node(
            package='risbot2_node',
            executable='risbot2_node.py',
            output='screen'),
    ])
