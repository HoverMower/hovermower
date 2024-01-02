import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python import get_package_prefix


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # Set the path to the world file
    world_file_name = 'garden.world'

    rsp_package_name='hovermower_bringup'
    pkg_share_path = get_package_share_directory('hovermower_simulation')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(rsp_package_name),'launch','hovermower_state_publisher.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Fix model path for Gazebo
    robot_share_path = os.pathsep + os.path.join(get_package_prefix("hovermower_description"), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += robot_share_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  robot_share_path
   
    # Set the path to the SDF model files.
    gazebo_models_path = os.pathsep + os.path.join(pkg_share_path, 'models')
    os.environ['GAZEBO_MODEL_PATH'] += gazebo_models_path

    print( os.environ['GAZEBO_MODEL_PATH'] )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'hovermower'],
                        output='screen')

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True
        }])

    twist_mux_params = os.path.join(get_package_share_directory(rsp_package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/cmd_vel')]
        )
    
    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_share_path, 'worlds', world_file_name), ''],
        description='SDF world file',
        ),
        rsp,
        joint_state_publisher, 
        spawn_entity,
        twist_mux,
        gazebo
    ])
