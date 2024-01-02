import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python import get_package_prefix


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    pkg_gazebo = get_package_share_directory('hovermower_simulation')

   
    # Set the path to the SDF model files.
    gazebo_models_path = os.pathsep + os.path.join(pkg_gazebo, 'models')
  
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_models_path
    

    print( os.environ['GAZEBO_MODEL_PATH'] )
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

   
    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_gazebo, 'worlds', 'garden_world.world'), ''],
        description='SDF world file',
        ),
        gazebo,
    ])
