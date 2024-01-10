import os

import ament_index_python.packages
import launch
import launch_ros.actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('hovermower_bringup'),
        'config')
    params = os.path.join(config_directory, 'gps.yaml')
    ublox_gps_node = launch_ros.actions.Node(package='ublox_gps',
                                             executable='ublox_gps_node',
                                             output='both',
                                             parameters=[params])

    params_ntrip_client = os.path.join(config_directory, 'ntrip_client.yaml')
    ntrip_client = Node(
                package='ntrip_client',
                executable='ntrip_ros.py',
                parameters=[params_ntrip_client],
                # Uncomment the following section and replace "/gx5/nmea/sentence" with the topic you are sending NMEA on if it is not the one we requested
                #remappings=[
                #  ("nmea", "/gx5/nmea/sentence")
                #],
          )

    
    return launch.LaunchDescription([ublox_gps_node,
                                     ntrip_client,
                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=ublox_gps_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])