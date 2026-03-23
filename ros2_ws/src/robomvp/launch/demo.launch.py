"""Plik uruchomieniowy dla systemu RoboMVP."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory('robomvp')
    config_dir = os.path.join(package_share_dir, 'config')
    scene_config = os.path.join(config_dir, 'scene.yaml')
    camera_config = os.path.join(config_dir, 'camera.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('marker_type', default_value='apriltag'),
        DeclareLaunchArgument('network_interface', default_value='eth0'),
        DeclareLaunchArgument('require_robot_connection', default_value='false'),
        DeclareLaunchArgument('body_camera_device', default_value='0'),
        DeclareLaunchArgument('head_camera_device', default_value='-1'),

        Node(package='robomvp', executable='camera_interface', name='camera_interface',
             output='screen', parameters=[{
                 'publish_rate': 10.0,
                 'body_camera_device': LaunchConfiguration('body_camera_device'),
                 'head_camera_device': LaunchConfiguration('head_camera_device'),
             }]),
        Node(package='robomvp', executable='marker_detection', name='marker_detection',
             output='screen', parameters=[{
                 'marker_type': LaunchConfiguration('marker_type'),
             }]),
        Node(package='robomvp', executable='marker_pose_estimator', name='marker_pose_estimator',
             output='screen', parameters=[{
                 'camera_config_path': camera_config,
                 'marker_size': 0.1,
             }]),
        Node(package='robomvp', executable='robomvp_main', name='robomvp_main',
             output='screen', parameters=[{
                 'scene_config_path': scene_config,
                 'step_period': 1.0,
                 'network_interface': LaunchConfiguration('network_interface'),
                 'require_robot_connection': LaunchConfiguration('require_robot_connection'),
             }]),
    ])
