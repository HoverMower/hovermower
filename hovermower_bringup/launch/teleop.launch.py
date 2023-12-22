#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import SetRemap
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    teleop_node = Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_node',
            output='screen',
            prefix = 'xterm -e',
            parameters=[ {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel','/cmd_vel_key')]
         )

    ds4_joy = IncludeLaunchDescription(
                XMLLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ds4_driver'),'launch','ds4_twist.launch.xml'
                )]), launch_arguments={'use_sim_time': use_sim_time,
                                       'dof': '2' }.items()           
                )

    twist_mux_params = os.path.join(get_package_share_directory('hovermower_bringup'),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/hoverboard_base_controller/cmd_vel_unstamped')]
        )    
    robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/hovermower_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )    
    
    hoverboard = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('hoverboard_driver'),'launch','diffbot.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    hovermower_base_controller = IncludeLaunchDescription(
                XMLLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_hovermower_base_controller'),'launch','base_controller.launch.xml'
                )]), launch_arguments={'use_sim_time': 'true',
                                       'device_port' : '/dev/ttyUSB0'}.items()
    )   
  
    bno08x = IncludeLaunchDescription(
                XMLLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_bno08x'),'launch','bno08x.launch.xml'
                )]), launch_arguments={'use_sim_time': 'true',
                                       'port' : '/dev/ttyUSB1'}.items()
    ) 
        
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        robot,
        teleop_node,
        ds4_joy,
        twist_mux,
        hoverboard,
        hovermower_base_controller,
        bno08x
        # twist_stamper       
    ])    