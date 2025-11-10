#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments (only for launch behavior control)
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Whether to launch USB camera'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz for visualization'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('tools_recognition'),
            'config',
            'yolov8_object_recognition.yaml'
        ]),
        description='Path to config file'
    )

    # USB Camera Launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usb_cam'),
                'launch',
                'camera.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('use_camera'))
    )

    # YOLOv8 Recognition Node
    yolov8_node = Node(
        package='tools_recognition',
        executable='yolov8_recognition',
        name='yolov8_recognition',
        parameters=[
            LaunchConfiguration('config_file'),
        ],
        output='screen',
        emulate_tty=False,
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('tools_recognition'),
            'rviz',
            'tools_recognition.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
        emulate_tty=False,
    )

    return LaunchDescription([
        use_camera_arg,
        use_rviz_arg,
        config_file_arg,
        camera_launch,
        yolov8_node,
        rviz_node,
    ])