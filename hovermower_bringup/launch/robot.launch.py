#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():

    hm_param_dir = LaunchConfiguration(
        'hm_param_dir',
        default=os.path.join(
            get_package_share_directory('hovermower_bringup'),
            'param',
            'hovermower.yaml'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'hm_param_dir',
            default_value=hm_param_dir,
            description='Full path to hovermower parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/hovermower_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

    ])