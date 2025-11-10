#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Declare launch arguments (only for launch behavior control)
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Whether to launch RealSense camera'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz for visualization'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('tools_recognition'),
            'config',
            'yolov8_object_tracking.yaml'
        ]),
        description='Path to config file'
    )

    # RealSense Camera Launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': 'micky_vision',
            'enable_rgbd': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'pointcloud.enable': 'true'
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_camera'))
    )

    # YOLOv8 Tracking Node
    yolov8_node = Node(
        package='tools_recognition',
        executable='object_tracking_node',
        name='object_tracking',
        parameters=[
            LaunchConfiguration('config_file'),
        ],
        output='screen',
        emulate_tty=False,
    )

    return LaunchDescription([
        use_camera_arg,
        use_rviz_arg,
        config_file_arg,
        camera_launch,
        yolov8_node,
    ])